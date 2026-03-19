/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#include <msgpack.h>
#include <nng/nng.h>
#include <nng/protocol/reqrep0/req.h>

#include "platform.h"

#include "target/SITL/sim/projectairsim.h"

#define PAS_DEFAULT_SERVICE_PORT 8990
#define PAS_METHOD_PATH_LEN 256
#define PAS_OBJECT_PRINT_BUFLEN 2048

typedef struct {
    msgpack_unpacked outer;
    msgpack_unpacked decoded;
    bool isError;
} pasParsedResponse_t;

static int pasRequestId = 0;

static bool pasPackKey(msgpack_packer *pk, const char *key)
{
    return msgpack_pack_str_with_body(pk, key, strlen(key)) == 0;
}

static bool pasPackStringValue(msgpack_packer *pk, const char *value)
{
    return msgpack_pack_str_with_body(pk, value, strlen(value)) == 0;
}

static bool pasPackBinaryValue(msgpack_packer *pk, const void *value, size_t length)
{
    return msgpack_pack_bin_with_body(pk, value, length) == 0;
}

static bool pasPackMethodValue(msgpack_packer *pk, const char *method)
{
    return pasPackBinaryValue(pk, method, strlen(method));
}

static bool pasReadSceneConfig(char **sceneConfig, size_t *sceneConfigLength)
{
    FILE *file = fopen(PROJECTAIRSIM_SCENE_PATH, "rb");
    if (file == NULL) {
        fprintf(stderr, "[PAS] scene_path=%s fopen_errno=%d\n", PROJECTAIRSIM_SCENE_PATH, errno);
        return false;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        fprintf(stderr, "[PAS] scene_path=%s fseek_errno=%d\n", PROJECTAIRSIM_SCENE_PATH, errno);
        fclose(file);
        return false;
    }

    long fileLength = ftell(file);
    if (fileLength <= 0) {
        fprintf(stderr, "[PAS] scene_path=%s file_length=%ld\n", PROJECTAIRSIM_SCENE_PATH, fileLength);
        fclose(file);
        return false;
    }

    if (fseek(file, 0, SEEK_SET) != 0) {
        fprintf(stderr, "[PAS] scene_path=%s rewind_errno=%d\n", PROJECTAIRSIM_SCENE_PATH, errno);
        fclose(file);
        return false;
    }

    char *buffer = calloc((size_t)fileLength + 1, sizeof(char));
    if (buffer == NULL) {
        fprintf(stderr, "[PAS] scene_path=%s alloc_bytes=%ld errno=%d\n", PROJECTAIRSIM_SCENE_PATH, fileLength + 1, errno);
        fclose(file);
        return false;
    }

    size_t bytesRead = fread(buffer, 1, (size_t)fileLength, file);
    fclose(file);

    if (bytesRead != (size_t)fileLength) {
        fprintf(stderr, "[PAS] scene_path=%s bytes_read=%zu expected_bytes=%ld\n", PROJECTAIRSIM_SCENE_PATH, bytesRead, fileLength);
        free(buffer);
        return false;
    }

    *sceneConfig = buffer;
    *sceneConfigLength = bytesRead;
    return true;
}

static bool pasObjectKeyEquals(const msgpack_object *object, const char *key)
{
    const size_t keyLength = strlen(key);

    if ((object->type == MSGPACK_OBJECT_STR) && (object->via.str.size == keyLength)) {
        return memcmp(object->via.str.ptr, key, keyLength) == 0;
    }

    if ((object->type == MSGPACK_OBJECT_BIN) && (object->via.bin.size == keyLength)) {
        return memcmp(object->via.bin.ptr, key, keyLength) == 0;
    }

    return false;
}

static const msgpack_object *pasFindMapValue(const msgpack_object *object, const char *key)
{
    if (object->type != MSGPACK_OBJECT_MAP) {
        return NULL;
    }

    for (uint32_t i = 0; i < object->via.map.size; i++) {
        const msgpack_object_kv *entry = &object->via.map.ptr[i];
        if (pasObjectKeyEquals(&entry->key, key)) {
            return &entry->val;
        }
    }

    return NULL;
}

static bool pasDecodeNestedObject(const msgpack_object *encodedObject, msgpack_unpacked *decodedObject)
{
    const char *buffer = NULL;
    size_t bufferLength = 0;

    if (encodedObject->type == MSGPACK_OBJECT_BIN) {
        buffer = encodedObject->via.bin.ptr;
        bufferLength = encodedObject->via.bin.size;
    } else if (encodedObject->type == MSGPACK_OBJECT_STR) {
        buffer = encodedObject->via.str.ptr;
        bufferLength = encodedObject->via.str.size;
    } else {
        return false;
    }

    size_t offset = 0;
    msgpack_unpacked_init(decodedObject);
    msgpack_unpack_return unpackResult = msgpack_unpack_next(decodedObject, buffer, bufferLength, &offset);
    return (unpackResult == MSGPACK_UNPACK_SUCCESS) || (unpackResult == MSGPACK_UNPACK_EXTRA_BYTES);
}

static bool pasPackRequest(msgpack_sbuffer *requestBuffer, const char *method, msgpack_sbuffer *paramsBuffer)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, requestBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 4) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "method") || !pasPackMethodValue(&packer, method)) {
        return false;
    }

    if (!pasPackKey(&packer, "params")) {
        return false;
    }
    if (msgpack_pack_map(&packer, 1) != 0) {
        return false;
    }
    if (!pasPackKey(&packer, "data") || !pasPackBinaryValue(&packer, paramsBuffer->data, paramsBuffer->size)) {
        return false;
    }

    if (!pasPackKey(&packer, "version") || (msgpack_pack_double(&packer, 1.0) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "id") || (msgpack_pack_int(&packer, pasRequestId++) != 0)) {
        return false;
    }

    return true;
}

static bool pasSendRequest(nng_socket socket, const char *method, msgpack_sbuffer *paramsBuffer, char **responseBuffer, size_t *responseLength)
{
    msgpack_sbuffer requestBuffer;
    msgpack_sbuffer_init(&requestBuffer);

    if (!pasPackRequest(&requestBuffer, method, paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s request_pack_failed=1\n", method);
        msgpack_sbuffer_destroy(&requestBuffer);
        return false;
    }

    int sendResult = nng_send(socket, requestBuffer.data, requestBuffer.size, 0);
    msgpack_sbuffer_destroy(&requestBuffer);

    if (sendResult != 0) {
        fprintf(stderr, "[PAS] method=%s nng_send_error=%d nng_send_error_text=%s\n", method, sendResult, nng_strerror(sendResult));
        return false;
    }

    void *recvBuffer = NULL;
    size_t recvLength = 0;
    int recvResult = nng_recv(socket, &recvBuffer, &recvLength, NNG_FLAG_ALLOC);
    if (recvResult != 0) {
        fprintf(stderr, "[PAS] method=%s nng_recv_error=%d nng_recv_error_text=%s\n", method, recvResult, nng_strerror(recvResult));
        return false;
    }

    *responseBuffer = recvBuffer;
    *responseLength = recvLength;
    return true;
}

static bool pasParseResponse(const char *method, const char *responseBuffer, size_t responseLength, pasParsedResponse_t *parsedResponse)
{
    memset(parsedResponse, 0, sizeof(*parsedResponse));

    msgpack_unpacked_init(&parsedResponse->outer);
    size_t responseOffset = 0;
    msgpack_unpack_return unpackResult = msgpack_unpack_next(&parsedResponse->outer, responseBuffer, responseLength, &responseOffset);
    if ((unpackResult != MSGPACK_UNPACK_SUCCESS) && (unpackResult != MSGPACK_UNPACK_EXTRA_BYTES)) {
        fprintf(stderr, "[PAS] method=%s response_unpack_result=%d response_bytes=%zu\n", method, unpackResult, responseLength);
        return false;
    }

    const msgpack_object *resultObject = pasFindMapValue(&parsedResponse->outer.data, "result");
    const msgpack_object *errorObject = pasFindMapValue(&parsedResponse->outer.data, "error");
    const msgpack_object *payloadObject = resultObject;

    if (errorObject != NULL) {
        parsedResponse->isError = true;
        payloadObject = errorObject;
    }

    if (payloadObject == NULL) {
        fprintf(stderr, "[PAS] method=%s response_has_result=%d response_has_error=%d\n", method, resultObject != NULL, errorObject != NULL);
        return false;
    }

    const msgpack_object *dataObject = pasFindMapValue(payloadObject, "data");
    if (dataObject == NULL) {
        fprintf(stderr, "[PAS] method=%s payload_type=%d payload_has_data=%d\n", method, payloadObject->type, dataObject != NULL);
        return false;
    }

    if (!pasDecodeNestedObject(dataObject, &parsedResponse->decoded)) {
        fprintf(stderr, "[PAS] method=%s data_type=%d data_decode_failed=1\n", method, dataObject->type);
        return false;
    }

    return true;
}

static void pasDestroyParsedResponse(pasParsedResponse_t *parsedResponse)
{
    msgpack_unpacked_destroy(&parsedResponse->decoded);
    msgpack_unpacked_destroy(&parsedResponse->outer);
}

static bool pasCopyObjectString(const msgpack_object *object, char *buffer, size_t bufferLength)
{
    const char *stringData = NULL;
    size_t stringLength = 0;

    if (object->type == MSGPACK_OBJECT_STR) {
        stringData = object->via.str.ptr;
        stringLength = object->via.str.size;
    } else if (object->type == MSGPACK_OBJECT_BIN) {
        stringData = object->via.bin.ptr;
        stringLength = object->via.bin.size;
    } else {
        return false;
    }

    if ((stringLength + 1) > bufferLength) {
        return false;
    }

    memcpy(buffer, stringData, stringLength);
    buffer[stringLength] = '\0';
    return true;
}

static bool pasObjectToInt64(const msgpack_object *object, int64_t *value)
{
    if (object->type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
        *value = (int64_t)object->via.u64;
        return true;
    }

    if (object->type == MSGPACK_OBJECT_NEGATIVE_INTEGER) {
        *value = object->via.i64;
        return true;
    }

    return false;
}

static void pasPrintObject(const char *method, const char *label, const msgpack_object *object)
{
    char printBuffer[PAS_OBJECT_PRINT_BUFLEN];
    int printLength = msgpack_object_print_buffer(printBuffer, sizeof(printBuffer), *object);

    if ((printLength < 0) || ((size_t)printLength >= sizeof(printBuffer))) {
        fprintf(stderr, "[PAS] method=%s %s_print_failed=1 object_type=%d\n", method, label, object->type);
        return;
    }

    fprintf(stderr, "[PAS] method=%s %s=%s\n", method, label, printBuffer);
}

static bool pasPackEmptyParams(msgpack_sbuffer *paramsBuffer)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);
    return msgpack_pack_map(&packer, 0) == 0;
}

static bool pasPackLoadSceneParams(msgpack_sbuffer *paramsBuffer, const char *sceneConfig)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 1) != 0) {
        return false;
    }

    return pasPackKey(&packer, "scene_config") && pasPackStringValue(&packer, sceneConfig);
}

static bool pasPackContinueForNStepsParams(msgpack_sbuffer *paramsBuffer, int nSteps)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 2) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "n_steps") || (msgpack_pack_int(&packer, nSteps) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "wait_until_complete") || (msgpack_pack_true(&packer) != 0)) {
        return false;
    }

    return true;
}

static bool pasLoadScene(nng_socket socket, const char *sceneConfig, char *sceneIdBuffer, size_t sceneIdBufferLength)
{
    const char *method = "/Sim/LoadScene";
    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackLoadSceneParams(&paramsBuffer, sceneConfig)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", method);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(method, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    bool copyOk = pasCopyObjectString(&parsedResponse.decoded.data, sceneIdBuffer, sceneIdBufferLength);
    if (!copyOk) {
        fprintf(stderr, "[PAS] method=%s scene_id_type=%d scene_id_buffer_length=%zu\n", method, parsedResponse.decoded.data.type, sceneIdBufferLength);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasContinueForNSteps(nng_socket socket, const char *worldPath, int nSteps, int64_t *simTimeNs)
{
    char method[PAS_METHOD_PATH_LEN];
    snprintf(method, sizeof(method), "%s/ContinueForNSteps", worldPath);

    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackContinueForNStepsParams(&paramsBuffer, nSteps)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1 n_steps=%d\n", method, nSteps);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(method, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    bool timeOk = pasObjectToInt64(&parsedResponse.decoded.data, simTimeNs);
    if (!timeOk) {
        fprintf(stderr, "[PAS] method=%s sim_time_type=%d n_steps=%d\n", method, parsedResponse.decoded.data.type, nSteps);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasFetchImuData(nng_socket socket, const char *imuPath)
{
    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackEmptyParams(&paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", imuPath);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    bool requestOk = pasSendRequest(socket, imuPath, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    bool parseOk = pasParseResponse(imuPath, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(imuPath, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if ((parsedResponse.decoded.data.type != MSGPACK_OBJECT_MAP) || (parsedResponse.decoded.data.via.map.size == 0)) {
        fprintf(stderr, "[PAS] method=%s imu_type=%d imu_map_size=%u\n", imuPath, parsedResponse.decoded.data.type, parsedResponse.decoded.data.type == MSGPACK_OBJECT_MAP ? parsedResponse.decoded.data.via.map.size : 0);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasPrintObject(imuPath, "imu_data", &parsedResponse.decoded.data);

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

bool simProjectAirSimInit(char *ip, int port, bool imu)
{
    (void)imu;

    char *sceneConfig = NULL;
    size_t sceneConfigLength = 0;
    if (!pasReadSceneConfig(&sceneConfig, &sceneConfigLength)) {
        return false;
    }

    nng_socket socket;
    int socketResult = nng_req0_open(&socket);
    if (socketResult != 0) {
        fprintf(stderr, "[PAS] req0_open_error=%d req0_open_error_text=%s\n", socketResult, nng_strerror(socketResult));
        free(sceneConfig);
        return false;
    }

    int timeoutResult = nng_socket_set_ms(socket, NNG_OPT_SENDTIMEO, 1000);
    if (timeoutResult == 0) {
        timeoutResult = nng_socket_set_ms(socket, NNG_OPT_RECVTIMEO, 10000);
    }
    if (timeoutResult != 0) {
        fprintf(stderr, "[PAS] socket_timeout_error=%d socket_timeout_error_text=%s\n", timeoutResult, nng_strerror(timeoutResult));
        nng_close(socket);
        free(sceneConfig);
        return false;
    }

    const int servicePort = (port > 0) ? port : PAS_DEFAULT_SERVICE_PORT;
    char serviceUrl[64];
    snprintf(serviceUrl, sizeof(serviceUrl), "tcp://%s:%d", ip, servicePort);

    int dialResult = nng_dial(socket, serviceUrl, NULL, 0);
    if (dialResult != 0) {
        fprintf(stderr, "[PAS] service_url=%s nng_dial_error=%d nng_dial_error_text=%s\n", serviceUrl, dialResult, nng_strerror(dialResult));
        nng_close(socket);
        free(sceneConfig);
        return false;
    }

    char sceneId[64];
    if (!pasLoadScene(socket, sceneConfig, sceneId, sizeof(sceneId))) {
        nng_close(socket);
        free(sceneConfig);
        return false;
    }

    free(sceneConfig);

    char worldPath[PAS_METHOD_PATH_LEN];
    char imuPath[PAS_METHOD_PATH_LEN];
    snprintf(worldPath, sizeof(worldPath), "/Sim/%s", sceneId);
    snprintf(imuPath, sizeof(imuPath), "%s/robots/InavQuad/sensors/IMU1/imu_kinematics", worldPath);

    int64_t simTimeNs = 0;
    if (!pasContinueForNSteps(socket, worldPath, 1, &simTimeNs)) {
        nng_close(socket);
        return false;
    }

    fprintf(stderr, "[PAS] service_url=%s scene_id=%s world_path=%s sim_time_ns=%" PRId64 " imu_path=%s\n", serviceUrl, sceneId, worldPath, simTimeNs, imuPath);

    bool imuOk = pasFetchImuData(socket, imuPath);
    nng_close(socket);
    return imuOk;
}
