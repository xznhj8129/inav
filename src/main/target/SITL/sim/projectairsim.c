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
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
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
#include <math.h>
#include <pthread.h>

#include <msgpack.h>
#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/protocol/reqrep0/req.h>

#include "platform.h"

#include "target.h"
#include "target/SITL/sim/projectairsim.h"
#include "target/SITL/sim/simHelper.h"

#include "common/maths.h"
#include "common/quaternion.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "io/gps.h"
#include "sensors/acceleration.h"
#include "sensors/battery_sensor_fake.h"

#define PAS_DEFAULT_SERVICE_PORT 8990
#define PAS_DEFAULT_TOPIC_PORT 8989
#define PAS_METHOD_PATH_LEN 256
#define PAS_OBJECT_PRINT_BUFLEN 2048

#define PAS_SEND_TIMEOUT_MS 1000
#define PAS_RECV_TIMEOUT_MS 10000
#define PAS_INIT_WAIT_MS 10000
#define PAS_TOPIC_WAIT_MS 1000
#define PAS_FAILURE_DELAY_US 10000
#define PAS_CONTROL_OUTPUT_COUNT 4
#define PAS_LOG_PERIOD 500
#define PAS_TOPIC_FRAME_SUBSCRIBE 0
#define PAS_TOPIC_FRAME_MESSAGE 2

#define PAS_SCENE_ROBOT_NAME "InavQuad"
#define PAS_SENSOR_IMU "IMU1"
#define PAS_SENSOR_GPS "GPS"
#define PAS_SENSOR_BAROMETER "Barometer"
#define PAS_SENSOR_MAGNETOMETER "Magnetometer"

#define PAS_ACTUATOR_FR "Prop_FR_actuator"
#define PAS_ACTUATOR_RL "Prop_RL_actuator"
#define PAS_ACTUATOR_FL "Prop_FL_actuator"
#define PAS_ACTUATOR_RR "Prop_RR_actuator"

#define PAS_FRAME_HALF_HEIGHT_M 0.02f
#define PAS_GROUND_PROBE_START_Z_M 0.05f
#define PAS_DEFAULT_TEMP_C 21
#define PAS_DEFAULT_VBAT_CENTIVOLTS 1680
#define PAS_SCENE_STEP_NS_KEY "\"step-ns\": "
#define PAS_SCENE_REALTIME_UPDATE_RATE_KEY "\"real-time-update-rate\": "
#define PAS_FAST_STEP_NS_TEXT "8000000"

typedef struct {
    msgpack_unpacked outer;
    msgpack_unpacked decoded;
    bool isError;
} pasParsedResponse_t;

typedef struct {
    float x;
    float y;
    float z;
} pasVector3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} pasQuaternion_t;

typedef struct {
    int64_t timeStamp;
    pasQuaternion_t orientation;
    pasVector3_t angularVelocity;
    pasVector3_t linearAcceleration;
} pasImuData_t;

typedef struct {
    int64_t timeStamp;
    int64_t timeUtcMillis;
    float latitude;
    float longitude;
    float altitude;
    float epv;
    float eph;
    int32_t positionCovType;
    int32_t fixType;
    pasVector3_t velocity;
} pasGpsData_t;

typedef struct {
    int64_t timeStamp;
    float altitude;
    float pressure;
    float qnh;
} pasBarometerData_t;

typedef struct {
    int64_t timeStamp;
    pasVector3_t magneticFieldBody;
} pasMagnetometerData_t;

typedef struct {
    pasVector3_t position;
    pasQuaternion_t orientation;
} pasPoseData_t;

typedef struct {
    int64_t timeStamp;
    pasPoseData_t pose;
} pasPoseStampedData_t;

typedef struct {
    int64_t timeStamp;
    pasPoseData_t pose;
    pasVector3_t linearAccelerationWorld;
} pasKinematicsData_t;

typedef struct {
    nng_socket socket;
    nng_socket topicSocket;
    pthread_t workerThread;
    bool running;
    bool initialized;
    bool useImu;
    bool havePrevAttitude;
    bool prevArmed;
    char serviceUrl[64];
    char topicUrl[64];
    char sceneId[64];
    char worldPath[PAS_METHOD_PATH_LEN];
    char robotPath[PAS_METHOD_PATH_LEN];
    char imuPath[PAS_METHOD_PATH_LEN];
    char gpsPath[PAS_METHOD_PATH_LEN];
    char barometerPath[PAS_METHOD_PATH_LEN];
    char magnetometerPath[PAS_METHOD_PATH_LEN];
    char actualPosePath[PAS_METHOD_PATH_LEN];
    pasImuData_t imuData;
    pasPoseStampedData_t actualPoseData;
    float groundZ;
    int16_t prevRoll;
    int16_t prevPitch;
    int16_t prevYaw;
    int64_t prevTimeStampNs;
    uint64_t loopCount;
    fpVector3_t lastFakeAcc;
    pasVector3_t lastImuLinearAcceleration;
    pasVector3_t lastGtLinearAccelerationWorld;
    pasGpsData_t gpsData;
    pasBarometerData_t barometerData;
    pasMagnetometerData_t magnetometerData;
    bool haveImuData;
    bool haveActualPoseData;
    bool haveGpsData;
    bool haveBarometerData;
    bool haveMagnetometerData;
} pasContext_t;

static int pasRequestId = 0;
static pasContext_t pasCtx;

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

    const long fileLength = ftell(file);
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

    const size_t bytesRead = fread(buffer, 1, (size_t)fileLength, file);
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

static bool pasOverwriteSceneNumericValue(char *sceneConfig, const char *key, const char *valueText)
{
    char *keyPos = strstr(sceneConfig, key);
    if (keyPos == NULL) {
        fprintf(stderr, "[PAS] scene_key_missing=%s\n", key);
        return false;
    }

    char *valuePos = keyPos + strlen(key);
    memcpy(valuePos, valueText, strlen(valueText));
    return true;
}

static bool pasApplyFastSceneConfig(char *sceneConfig)
{
    return pasOverwriteSceneNumericValue(sceneConfig, PAS_SCENE_STEP_NS_KEY, PAS_FAST_STEP_NS_TEXT) &&
           pasOverwriteSceneNumericValue(sceneConfig, PAS_SCENE_REALTIME_UPDATE_RATE_KEY, PAS_FAST_STEP_NS_TEXT);
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
    const msgpack_unpack_return unpackResult = msgpack_unpack_next(decodedObject, buffer, bufferLength, &offset);
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

    const int sendResult = nng_send(socket, requestBuffer.data, requestBuffer.size, 0);
    msgpack_sbuffer_destroy(&requestBuffer);

    if (sendResult != 0) {
        fprintf(stderr, "[PAS] method=%s nng_send_error=%d nng_send_error_text=%s\n", method, sendResult, nng_strerror(sendResult));
        return false;
    }

    void *recvBuffer = NULL;
    size_t recvLength = 0;
    const int recvResult = nng_recv(socket, &recvBuffer, &recvLength, NNG_FLAG_ALLOC);
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
    const msgpack_unpack_return unpackResult = msgpack_unpack_next(&parsedResponse->outer, responseBuffer, responseLength, &responseOffset);
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

static void pasPrintObject(const char *method, const char *label, const msgpack_object *object)
{
    char printBuffer[PAS_OBJECT_PRINT_BUFLEN];
    const int printLength = msgpack_object_print_buffer(printBuffer, sizeof(printBuffer), *object);

    if ((printLength < 0) || ((size_t)printLength >= sizeof(printBuffer))) {
        fprintf(stderr, "[PAS] method=%s %s_print_failed=1 object_type=%d\n", method, label, object->type);
        return;
    }

    fprintf(stderr, "[PAS] method=%s %s=%s\n", method, label, printBuffer);
}

static bool pasObjectToFloat(const msgpack_object *object, float *value)
{
    switch (object->type) {
        case MSGPACK_OBJECT_POSITIVE_INTEGER:
            *value = (float)object->via.u64;
            return true;
        case MSGPACK_OBJECT_NEGATIVE_INTEGER:
            *value = (float)object->via.i64;
            return true;
        case MSGPACK_OBJECT_FLOAT32:
        case MSGPACK_OBJECT_FLOAT64:
            *value = (float)object->via.f64;
            return true;
        default:
            return false;
    }
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

static bool pasObjectToInt32(const msgpack_object *object, int32_t *value)
{
    int64_t intValue = 0;
    if (!pasObjectToInt64(object, &intValue)) {
        return false;
    }

    *value = (int32_t)intValue;
    return true;
}

static bool pasObjectToVector3(const msgpack_object *object, pasVector3_t *vector)
{
    if (object->type == MSGPACK_OBJECT_ARRAY) {
        if (object->via.array.size != 3) {
            return false;
        }

        return
            pasObjectToFloat(&object->via.array.ptr[0], &vector->x) &&
            pasObjectToFloat(&object->via.array.ptr[1], &vector->y) &&
            pasObjectToFloat(&object->via.array.ptr[2], &vector->z);
    }

    const msgpack_object *x = pasFindMapValue(object, "x");
    const msgpack_object *y = pasFindMapValue(object, "y");
    const msgpack_object *z = pasFindMapValue(object, "z");

    return (x != NULL) && (y != NULL) && (z != NULL) &&
        pasObjectToFloat(x, &vector->x) &&
        pasObjectToFloat(y, &vector->y) &&
        pasObjectToFloat(z, &vector->z);
}

static bool pasObjectToQuaternion(const msgpack_object *object, pasQuaternion_t *quaternion)
{
    const msgpack_object *w = pasFindMapValue(object, "w");
    const msgpack_object *x = pasFindMapValue(object, "x");
    const msgpack_object *y = pasFindMapValue(object, "y");
    const msgpack_object *z = pasFindMapValue(object, "z");

    return (w != NULL) && (x != NULL) && (y != NULL) && (z != NULL) &&
        pasObjectToFloat(w, &quaternion->w) &&
        pasObjectToFloat(x, &quaternion->x) &&
        pasObjectToFloat(y, &quaternion->y) &&
        pasObjectToFloat(z, &quaternion->z);
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

static bool pasPackPoseParams(msgpack_sbuffer *paramsBuffer, float x, float y, float z, const pasQuaternion_t *orientation)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 2) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "pose")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 3) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "frame_id") || !pasPackStringValue(&packer, "DEFAULT_FRAME")) {
        return false;
    }

    if (!pasPackKey(&packer, "translation")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 3) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "x") || (msgpack_pack_double(&packer, x) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "y") || (msgpack_pack_double(&packer, y) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "z") || (msgpack_pack_double(&packer, z) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "rotation")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 4) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "w") || (msgpack_pack_double(&packer, orientation->w) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "x") || (msgpack_pack_double(&packer, orientation->x) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "y") || (msgpack_pack_double(&packer, orientation->y) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "z") || (msgpack_pack_double(&packer, orientation->z) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "reset_kinematics") || (msgpack_pack_true(&packer) != 0)) {
        return false;
    }

    return true;
}

static bool pasPackHitTestParams(msgpack_sbuffer *paramsBuffer, float x, float y, float z, const pasQuaternion_t *orientation)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 1) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "pose")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 3) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "frame_id") || !pasPackStringValue(&packer, "DEFAULT_FRAME")) {
        return false;
    }

    if (!pasPackKey(&packer, "translation")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 3) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "x") || (msgpack_pack_double(&packer, x) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "y") || (msgpack_pack_double(&packer, y) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "z") || (msgpack_pack_double(&packer, z) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "rotation")) {
        return false;
    }

    if (msgpack_pack_map(&packer, 4) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "w") || (msgpack_pack_double(&packer, orientation->w) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "x") || (msgpack_pack_double(&packer, orientation->x) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "y") || (msgpack_pack_double(&packer, orientation->y) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, "z") || (msgpack_pack_double(&packer, orientation->z) != 0)) {
        return false;
    }

    return true;
}

static bool pasPackControlSignalsParams(msgpack_sbuffer *paramsBuffer, const float motorOutputs[PAS_CONTROL_OUTPUT_COUNT])
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, paramsBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_map(&packer, 1) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, "control_signal_map")) {
        return false;
    }

    if (msgpack_pack_map(&packer, PAS_CONTROL_OUTPUT_COUNT) != 0) {
        return false;
    }

    if (!pasPackKey(&packer, PAS_ACTUATOR_RR) || (msgpack_pack_double(&packer, motorOutputs[0]) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, PAS_ACTUATOR_FR) || (msgpack_pack_double(&packer, motorOutputs[1]) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, PAS_ACTUATOR_RL) || (msgpack_pack_double(&packer, motorOutputs[2]) != 0)) {
        return false;
    }

    if (!pasPackKey(&packer, PAS_ACTUATOR_FL) || (msgpack_pack_double(&packer, motorOutputs[3]) != 0)) {
        return false;
    }

    return true;
}

static bool pasPackTopicSubscribeFrame(msgpack_sbuffer *frameBuffer, const char *topicPath)
{
    msgpack_packer packer;
    msgpack_packer_init(&packer, frameBuffer, msgpack_sbuffer_write);

    if (msgpack_pack_array(&packer, 3) != 0) {
        return false;
    }

    if ((msgpack_pack_int(&packer, PAS_TOPIC_FRAME_SUBSCRIBE) != 0) ||
        !pasPackStringValue(&packer, topicPath) ||
        !pasPackStringValue(&packer, "")) {
        return false;
    }

    return true;
}

static bool pasSubscribeTopic(nng_socket socket, const char *topicPath)
{
    msgpack_sbuffer frameBuffer;
    msgpack_sbuffer_init(&frameBuffer);

    if (!pasPackTopicSubscribeFrame(&frameBuffer, topicPath)) {
        fprintf(stderr, "[PAS] topic_path=%s subscribe_pack_failed=1\n", topicPath);
        msgpack_sbuffer_destroy(&frameBuffer);
        return false;
    }

    const int sendResult = nng_send(socket, frameBuffer.data, frameBuffer.size, 0);
    msgpack_sbuffer_destroy(&frameBuffer);

    if (sendResult != 0) {
        fprintf(stderr, "[PAS] topic_path=%s topic_send_error=%d topic_send_error_text=%s\n", topicPath, sendResult, nng_strerror(sendResult));
        return false;
    }

    return true;
}

static bool pasSubscribeTopics(pasContext_t *ctx)
{
    if (!pasSubscribeTopic(ctx->topicSocket, ctx->imuPath) ||
        !pasSubscribeTopic(ctx->topicSocket, ctx->gpsPath) ||
        !pasSubscribeTopic(ctx->topicSocket, ctx->barometerPath)) {
        return false;
    }

    if (ctx->useImu && !pasSubscribeTopic(ctx->topicSocket, ctx->magnetometerPath)) {
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
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    const msgpack_object *sceneIdObject = &parsedResponse.decoded.data;
    const char *stringData = NULL;
    size_t stringLength = 0;

    if (sceneIdObject->type == MSGPACK_OBJECT_STR) {
        stringData = sceneIdObject->via.str.ptr;
        stringLength = sceneIdObject->via.str.size;
    } else if (sceneIdObject->type == MSGPACK_OBJECT_BIN) {
        stringData = sceneIdObject->via.bin.ptr;
        stringLength = sceneIdObject->via.bin.size;
    }

    if ((stringData == NULL) || (stringLength + 1 > sceneIdBufferLength)) {
        fprintf(stderr, "[PAS] method=%s scene_id_type=%d scene_id_buffer_length=%zu\n", method, sceneIdObject->type, sceneIdBufferLength);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    memcpy(sceneIdBuffer, stringData, stringLength);
    sceneIdBuffer[stringLength] = '\0';

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
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    const bool timeOk = pasObjectToInt64(&parsedResponse.decoded.data, simTimeNs);
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

static bool pasSetPose(nng_socket socket, const char *robotPath, float x, float y, float z, const pasQuaternion_t *orientation)
{
    char method[PAS_METHOD_PATH_LEN];
    snprintf(method, sizeof(method), "%s/SetPose", robotPath);

    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackPoseParams(&paramsBuffer, x, y, z, orientation)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1 x=%.3f y=%.3f z=%.3f\n", method, x, y, z);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasSetControlSignals(nng_socket socket, const char *robotPath, const float motorOutputs[PAS_CONTROL_OUTPUT_COUNT])
{
    char method[PAS_METHOD_PATH_LEN];
    snprintf(method, sizeof(method), "%s/SetControlSignals", robotPath);

    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackControlSignalsParams(&paramsBuffer, motorOutputs)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1 motor_rr=%.3f motor_fr=%.3f motor_rl=%.3f motor_fl=%.3f\n",
            method, motorOutputs[0], motorOutputs[1], motorOutputs[2], motorOutputs[3]);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasHitTest(nng_socket socket, const char *worldPath, float x, float y, float z, const pasQuaternion_t *orientation, pasVector3_t *hitPoint)
{
    char method[PAS_METHOD_PATH_LEN];
    snprintf(method, sizeof(method), "%s/HitTest", worldPath);

    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackHitTestParams(&paramsBuffer, x, y, z, orientation)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1 x=%.3f y=%.3f z=%.3f\n", method, x, y, z);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    const bool hitOk = pasObjectToVector3(&parsedResponse.decoded.data, hitPoint);
    if (!hitOk) {
        fprintf(stderr, "[PAS] method=%s hit_type=%d x=%.3f y=%.3f z=%.3f\n", method, parsedResponse.decoded.data.type, x, y, z);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasGetGroundZ(nng_socket socket, const char *worldPath, float x, float y, float *groundZ)
{
    const pasQuaternion_t downwardTraceOrientation = {
        .w = 0.70710678f,
        .x = 0.0f,
        .y = -0.70710678f,
        .z = 0.0f,
    };
    pasVector3_t hitPoint;

    if (!pasHitTest(socket, worldPath, x, y, PAS_GROUND_PROBE_START_Z_M, &downwardTraceOrientation, &hitPoint)) {
        return false;
    }

    if (!isfinite(hitPoint.z)) {
        fprintf(stderr, "[PAS] world_path=%s probe_x=%.3f probe_y=%.3f hit_z=%.3f\n", worldPath, x, y, hitPoint.z);
        return false;
    }

    *groundZ = hitPoint.z;
    return true;
}

static bool pasParseImuDataObject(const char *source, const msgpack_object *object, pasImuData_t *imuData)
{
    const msgpack_object *timeStampObject = pasFindMapValue(object, "time_stamp");
    const msgpack_object *orientationObject = pasFindMapValue(object, "orientation");
    const msgpack_object *angularVelocityObject = pasFindMapValue(object, "angular_velocity");
    const msgpack_object *linearAccelerationObject = pasFindMapValue(object, "linear_acceleration");

    const bool ok =
        (timeStampObject != NULL) &&
        (orientationObject != NULL) &&
        (angularVelocityObject != NULL) &&
        (linearAccelerationObject != NULL) &&
        pasObjectToInt64(timeStampObject, &imuData->timeStamp) &&
        pasObjectToQuaternion(orientationObject, &imuData->orientation) &&
        pasObjectToVector3(angularVelocityObject, &imuData->angularVelocity) &&
        pasObjectToVector3(linearAccelerationObject, &imuData->linearAcceleration);

    if (!ok) {
        pasPrintObject(source, "imu_data", object);
    }

    return ok;
}

static bool pasParsePoseStampedDataObject(const char *source, const msgpack_object *object, pasPoseStampedData_t *poseData)
{
    const msgpack_object *timeStampObject = pasFindMapValue(object, "time_stamp");
    const msgpack_object *positionObject = pasFindMapValue(object, "position");
    const msgpack_object *orientationObject = pasFindMapValue(object, "orientation");

    const bool ok =
        (timeStampObject != NULL) &&
        (positionObject != NULL) &&
        (orientationObject != NULL) &&
        pasObjectToInt64(timeStampObject, &poseData->timeStamp) &&
        pasObjectToVector3(positionObject, &poseData->pose.position) &&
        pasObjectToQuaternion(orientationObject, &poseData->pose.orientation);

    if (!ok) {
        pasPrintObject(source, "pose_stamped_data", object);
    }

    return ok;
}

static bool pasParseGpsDataObject(const char *source, const msgpack_object *object, pasGpsData_t *gpsData)
{
    const msgpack_object *timeStampObject = pasFindMapValue(object, "time_stamp");
    const msgpack_object *timeUtcMillisObject = pasFindMapValue(object, "time_utc_millis");
    const msgpack_object *latitudeObject = pasFindMapValue(object, "latitude");
    const msgpack_object *longitudeObject = pasFindMapValue(object, "longitude");
    const msgpack_object *altitudeObject = pasFindMapValue(object, "altitude");
    const msgpack_object *epvObject = pasFindMapValue(object, "epv");
    const msgpack_object *ephObject = pasFindMapValue(object, "eph");
    const msgpack_object *positionCovTypeObject = pasFindMapValue(object, "position_cov_type");
    const msgpack_object *fixTypeObject = pasFindMapValue(object, "fix_type");
    const msgpack_object *velocityObject = pasFindMapValue(object, "velocity");

    const bool ok =
        (timeStampObject != NULL) &&
        (timeUtcMillisObject != NULL) &&
        (latitudeObject != NULL) &&
        (longitudeObject != NULL) &&
        (altitudeObject != NULL) &&
        (epvObject != NULL) &&
        (ephObject != NULL) &&
        (positionCovTypeObject != NULL) &&
        (fixTypeObject != NULL) &&
        (velocityObject != NULL) &&
        pasObjectToInt64(timeStampObject, &gpsData->timeStamp) &&
        pasObjectToInt64(timeUtcMillisObject, &gpsData->timeUtcMillis) &&
        pasObjectToFloat(latitudeObject, &gpsData->latitude) &&
        pasObjectToFloat(longitudeObject, &gpsData->longitude) &&
        pasObjectToFloat(altitudeObject, &gpsData->altitude) &&
        pasObjectToFloat(epvObject, &gpsData->epv) &&
        pasObjectToFloat(ephObject, &gpsData->eph) &&
        pasObjectToInt32(positionCovTypeObject, &gpsData->positionCovType) &&
        pasObjectToInt32(fixTypeObject, &gpsData->fixType) &&
        pasObjectToVector3(velocityObject, &gpsData->velocity);

    if (!ok) {
        pasPrintObject(source, "gps_data", object);
    }

    return ok;
}

static bool pasParseBarometerDataObject(const char *source, const msgpack_object *object, pasBarometerData_t *barometerData)
{
    const msgpack_object *timeStampObject = pasFindMapValue(object, "time_stamp");
    const msgpack_object *altitudeObject = pasFindMapValue(object, "altitude");
    const msgpack_object *pressureObject = pasFindMapValue(object, "pressure");
    const msgpack_object *qnhObject = pasFindMapValue(object, "qnh");

    const bool ok =
        (timeStampObject != NULL) &&
        (altitudeObject != NULL) &&
        (pressureObject != NULL) &&
        (qnhObject != NULL) &&
        pasObjectToInt64(timeStampObject, &barometerData->timeStamp) &&
        pasObjectToFloat(altitudeObject, &barometerData->altitude) &&
        pasObjectToFloat(pressureObject, &barometerData->pressure) &&
        pasObjectToFloat(qnhObject, &barometerData->qnh);

    if (!ok) {
        pasPrintObject(source, "barometer_data", object);
    }

    return ok;
}

static bool pasParseMagnetometerDataObject(const char *source, const msgpack_object *object, pasMagnetometerData_t *magnetometerData)
{
    const msgpack_object *timeStampObject = pasFindMapValue(object, "time_stamp");
    const msgpack_object *fieldObject = pasFindMapValue(object, "magnetic_field_body");

    const bool ok =
        (timeStampObject != NULL) &&
        (fieldObject != NULL) &&
        pasObjectToInt64(timeStampObject, &magnetometerData->timeStamp) &&
        pasObjectToVector3(fieldObject, &magnetometerData->magneticFieldBody);

    if (!ok) {
        pasPrintObject(source, "magnetometer_data", object);
    }

    return ok;
}

static bool pasFetchImuData(nng_socket socket, const char *imuPath, pasImuData_t *imuData)
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
    const bool requestOk = pasSendRequest(socket, imuPath, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(imuPath, responseBuffer, responseLength, &parsedResponse);
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

    const bool ok = pasParseImuDataObject(imuPath, &parsedResponse.decoded.data, imuData);
    if (!ok) {
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasFetchGpsData(nng_socket socket, const char *gpsPath, pasGpsData_t *gpsData)
{
    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackEmptyParams(&paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", gpsPath);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, gpsPath, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(gpsPath, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(gpsPath, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    const bool ok = pasParseGpsDataObject(gpsPath, &parsedResponse.decoded.data, gpsData);
    if (!ok) {
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasFetchBarometerData(nng_socket socket, const char *barometerPath, pasBarometerData_t *barometerData)
{
    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackEmptyParams(&paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", barometerPath);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, barometerPath, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(barometerPath, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(barometerPath, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    const bool ok = pasParseBarometerDataObject(barometerPath, &parsedResponse.decoded.data, barometerData);
    if (!ok) {
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasFetchMagnetometerData(nng_socket socket, const char *magnetometerPath, pasMagnetometerData_t *magnetometerData)
{
    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackEmptyParams(&paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", magnetometerPath);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, magnetometerPath, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(magnetometerPath, responseBuffer, responseLength, &parsedResponse);
    if (!parseOk) {
        nng_free(responseBuffer, responseLength);
        return false;
    }

    if (parsedResponse.isError) {
        pasPrintObject(magnetometerPath, "error_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    const bool ok = pasParseMagnetometerDataObject(magnetometerPath, &parsedResponse.decoded.data, magnetometerData);
    if (!ok) {
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasFetchGroundTruthKinematics(nng_socket socket, const char *robotPath, pasKinematicsData_t *kinematicsData)
{
    char method[PAS_METHOD_PATH_LEN];
    snprintf(method, sizeof(method), "%s/GetGroundTruthKinematics", robotPath);

    msgpack_sbuffer paramsBuffer;
    msgpack_sbuffer_init(&paramsBuffer);

    if (!pasPackEmptyParams(&paramsBuffer)) {
        fprintf(stderr, "[PAS] method=%s params_pack_failed=1\n", method);
        msgpack_sbuffer_destroy(&paramsBuffer);
        return false;
    }

    char *responseBuffer = NULL;
    size_t responseLength = 0;
    const bool requestOk = pasSendRequest(socket, method, &paramsBuffer, &responseBuffer, &responseLength);
    msgpack_sbuffer_destroy(&paramsBuffer);
    if (!requestOk) {
        return false;
    }

    pasParsedResponse_t parsedResponse;
    const bool parseOk = pasParseResponse(method, responseBuffer, responseLength, &parsedResponse);
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

    const msgpack_object *timeStampObject = pasFindMapValue(&parsedResponse.decoded.data, "time_stamp");
    const msgpack_object *poseObject = pasFindMapValue(&parsedResponse.decoded.data, "pose");
    const msgpack_object *accelsObject = pasFindMapValue(&parsedResponse.decoded.data, "accels");
    const msgpack_object *positionObject = poseObject ? pasFindMapValue(poseObject, "position") : NULL;
    const msgpack_object *orientationObject = poseObject ? pasFindMapValue(poseObject, "orientation") : NULL;
    const msgpack_object *linearAccelerationObject = accelsObject ? pasFindMapValue(accelsObject, "linear") : NULL;

    const bool ok =
        (timeStampObject != NULL) &&
        (poseObject != NULL) &&
        (accelsObject != NULL) &&
        (positionObject != NULL) &&
        (orientationObject != NULL) &&
        (linearAccelerationObject != NULL) &&
        pasObjectToInt64(timeStampObject, &kinematicsData->timeStamp) &&
        pasObjectToVector3(positionObject, &kinematicsData->pose.position) &&
        pasObjectToQuaternion(orientationObject, &kinematicsData->pose.orientation) &&
        pasObjectToVector3(linearAccelerationObject, &kinematicsData->linearAccelerationWorld);

    if (!ok) {
        pasPrintObject(method, "kinematics_data", &parsedResponse.decoded.data);
        pasDestroyParsedResponse(&parsedResponse);
        nng_free(responseBuffer, responseLength);
        return false;
    }

    pasDestroyParsedResponse(&parsedResponse);
    nng_free(responseBuffer, responseLength);
    return true;
}

static bool pasHandleTopicFrame(pasContext_t *ctx, const void *frameBuffer, size_t frameLength)
{
    msgpack_unpacked frame;
    msgpack_unpacked_init(&frame);

    size_t offset = 0;
    const msgpack_unpack_return unpackResult = msgpack_unpack_next(&frame, frameBuffer, frameLength, &offset);
    if ((unpackResult != MSGPACK_UNPACK_SUCCESS) && (unpackResult != MSGPACK_UNPACK_EXTRA_BYTES)) {
        fprintf(stderr, "[PAS] topic_frame_unpack_result=%d topic_frame_bytes=%zu\n", unpackResult, frameLength);
        msgpack_unpacked_destroy(&frame);
        return false;
    }

    const msgpack_object *array = &frame.data;
    if ((array->type != MSGPACK_OBJECT_ARRAY) || (array->via.array.size != 3)) {
        fprintf(stderr, "[PAS] topic_frame_type=%d topic_frame_size=%u\n", array->type, array->type == MSGPACK_OBJECT_ARRAY ? array->via.array.size : 0);
        msgpack_unpacked_destroy(&frame);
        return false;
    }

    const msgpack_object *frameTypeObject = &array->via.array.ptr[0];
    const msgpack_object *topicObject = &array->via.array.ptr[1];
    const msgpack_object *payloadObject = &array->via.array.ptr[2];
    int64_t frameType = -1;

    if (!pasObjectToInt64(frameTypeObject, &frameType)) {
        fprintf(stderr, "[PAS] topic_frame_type_decode_failed=1\n");
        msgpack_unpacked_destroy(&frame);
        return false;
    }

    if (frameType != PAS_TOPIC_FRAME_MESSAGE) {
        msgpack_unpacked_destroy(&frame);
        return true;
    }

    const char *topicPath = NULL;
    size_t topicPathLength = 0;
    if (topicObject->type == MSGPACK_OBJECT_STR) {
        topicPath = topicObject->via.str.ptr;
        topicPathLength = topicObject->via.str.size;
    } else if (topicObject->type == MSGPACK_OBJECT_BIN) {
        topicPath = topicObject->via.bin.ptr;
        topicPathLength = topicObject->via.bin.size;
    }

    if (topicPath == NULL) {
        fprintf(stderr, "[PAS] topic_path_type=%d topic_path_decode_failed=1\n", topicObject->type);
        msgpack_unpacked_destroy(&frame);
        return false;
    }

    msgpack_unpacked payload;
    if (!pasDecodeNestedObject(payloadObject, &payload)) {
        fprintf(stderr, "[PAS] topic_payload_type=%d topic_payload_decode_failed=1\n", payloadObject->type);
        msgpack_unpacked_destroy(&frame);
        return false;
    }

    bool ok = true;
    if ((topicPathLength == strlen(ctx->imuPath)) && (memcmp(topicPath, ctx->imuPath, topicPathLength) == 0)) {
        ok = pasParseImuDataObject(ctx->imuPath, &payload.data, &ctx->imuData);
        ctx->haveImuData = ok;
    } else if ((topicPathLength == strlen(ctx->actualPosePath)) && (memcmp(topicPath, ctx->actualPosePath, topicPathLength) == 0)) {
        ok = pasParsePoseStampedDataObject(ctx->actualPosePath, &payload.data, &ctx->actualPoseData);
        ctx->haveActualPoseData = ok;
    } else if ((topicPathLength == strlen(ctx->gpsPath)) && (memcmp(topicPath, ctx->gpsPath, topicPathLength) == 0)) {
        ok = pasParseGpsDataObject(ctx->gpsPath, &payload.data, &ctx->gpsData);
        ctx->haveGpsData = ok;
    } else if ((topicPathLength == strlen(ctx->barometerPath)) && (memcmp(topicPath, ctx->barometerPath, topicPathLength) == 0)) {
        ok = pasParseBarometerDataObject(ctx->barometerPath, &payload.data, &ctx->barometerData);
        ctx->haveBarometerData = ok;
    } else if ((topicPathLength == strlen(ctx->magnetometerPath)) && (memcmp(topicPath, ctx->magnetometerPath, topicPathLength) == 0)) {
        ok = pasParseMagnetometerDataObject(ctx->magnetometerPath, &payload.data, &ctx->magnetometerData);
        ctx->haveMagnetometerData = ok;
    }

    msgpack_unpacked_destroy(&payload);
    msgpack_unpacked_destroy(&frame);
    return ok;
}

static bool pasDrainTopicSocket(pasContext_t *ctx)
{
    while (true) {
        void *recvBuffer = NULL;
        size_t recvLength = 0;
        const int recvResult = nng_recv(ctx->topicSocket, &recvBuffer, &recvLength, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);

        if (recvResult == NNG_EAGAIN) {
            return true;
        }

        if (recvResult != 0) {
            fprintf(stderr, "[PAS] topic_nng_recv_error=%d topic_nng_recv_error_text=%s\n", recvResult, nng_strerror(recvResult));
            return false;
        }

        const bool ok = pasHandleTopicFrame(ctx, recvBuffer, recvLength);
        nng_free(recvBuffer, recvLength);
        if (!ok) {
            return false;
        }
    }
}

static bool pasWaitForTopicData(pasContext_t *ctx, int64_t simTimeNs)
{
    const uint32_t waitStartMs = millis();

    while (true) {
        if (!pasDrainTopicSocket(ctx)) {
            return false;
        }

        if (ctx->haveImuData &&
            (ctx->imuData.timeStamp >= simTimeNs) &&
            ctx->haveGpsData &&
            ctx->haveBarometerData &&
            (!ctx->useImu || ctx->haveMagnetometerData)) {
            return true;
        }

        if ((millis() - waitStartMs) > PAS_TOPIC_WAIT_MS) {
            fprintf(stderr,
                "[PAS] topic_wait_timeout_ms=%d sim_time_ns=%" PRId64 " have_imu=%d imu_time_ns=%" PRId64 " have_gps=%d have_baro=%d have_mag=%d\n",
                PAS_TOPIC_WAIT_MS,
                simTimeNs,
                ctx->haveImuData,
                ctx->haveImuData ? ctx->imuData.timeStamp : -1,
                ctx->haveGpsData,
                ctx->haveBarometerData,
                ctx->haveMagnetometerData);
            return false;
        }

        delayMicroseconds(500);
    }
}

static int16_t pasWrap3600(int16_t angle)
{
    while (angle < 0) {
        angle += 3600;
    }

    while (angle >= 3600) {
        angle -= 3600;
    }

    return angle;
}

static void pasQuaternionToInavAttitude(const pasQuaternion_t *quaternion, int16_t *roll, int16_t *pitch, int16_t *yaw)
{
    const float sinrCosp = 2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
    const float cosrCosp = 1.0f - 2.0f * (quaternion->x * quaternion->x + quaternion->y * quaternion->y);
    const float standardRoll = atan2f(sinrCosp, cosrCosp);

    const float sinp = constrainf(2.0f * (quaternion->w * quaternion->y - quaternion->z * quaternion->x), -1.0f, 1.0f);
    const float standardPitch = asinf(sinp);

    const float sinyCosp = 2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y);
    const float cosyCosp = 1.0f - 2.0f * (quaternion->y * quaternion->y + quaternion->z * quaternion->z);
    const float standardYaw = atan2f(sinyCosp, cosyCosp);

    *roll = constrainToInt16(RADIANS_TO_DECIDEGREES(standardRoll));
    *pitch = constrainToInt16(RADIANS_TO_DECIDEGREES(-standardPitch));
    *yaw = pasWrap3600(constrainToInt16(RADIANS_TO_DECIDEGREES(-standardYaw)));
}

static void pasApplyAttitudeDerivedMag(int16_t roll, int16_t pitch, int16_t yaw)
{
    fpQuaternion_t quat;
    computeQuaternionFromRPY(&quat, roll, pitch, yaw);

    fpVector3_t north;
    north.x = 1.0f;
    north.y = 0.0f;
    north.z = 0.0f;
    transformVectorEarthToBody(&north, &quat);

    fakeMagSet(
        constrainToInt16(north.x * 1024.0f),
        constrainToInt16(north.y * 1024.0f),
        constrainToInt16(north.z * 1024.0f)
    );
}

static void pasApplyImuData(pasContext_t *ctx, const pasImuData_t *imuData, const pasPoseData_t *poseData)
{
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;
    const pasQuaternion_t *attitudeOrientation = &imuData->orientation;
    if (!ctx->useImu) {
        attitudeOrientation = &poseData->orientation;
    }
    pasQuaternionToInavAttitude(attitudeOrientation, &roll, &pitch, &yaw);

    if (!ctx->useImu) {
        imuSetAttitudeRPY(roll, pitch, yaw);
        imuUpdateAttitude(micros());
        pasApplyAttitudeDerivedMag(roll, pitch, yaw);
    }

    ctx->prevRoll = roll;
    ctx->prevPitch = pitch;
    ctx->prevYaw = yaw;
    ctx->prevTimeStampNs = imuData->timeStamp;
    ctx->havePrevAttitude = true;
    ctx->lastImuLinearAcceleration = imuData->linearAcceleration;

    fakeGyroSet(
        constrainToInt16(RADIANS_TO_DEGREES(imuData->angularVelocity.x) * 16.0f),
        constrainToInt16(-RADIANS_TO_DEGREES(imuData->angularVelocity.y) * 16.0f),
        constrainToInt16(-RADIANS_TO_DEGREES(imuData->angularVelocity.z) * 16.0f)
    );

    ctx->lastFakeAcc.x = -imuData->linearAcceleration.x * 1000.0f;
    ctx->lastFakeAcc.y = -imuData->linearAcceleration.y * 1000.0f;
    ctx->lastFakeAcc.z = -imuData->linearAcceleration.z * 1000.0f;

    fakeAccSet(
        constrainToInt16(ctx->lastFakeAcc.x),
        constrainToInt16(ctx->lastFakeAcc.y),
        constrainToInt16(ctx->lastFakeAcc.z)
    );
}

static void pasApplyGpsData(const pasGpsData_t *gpsData)
{
    gpsFixType_e fixType = GPS_NO_FIX;
    if (gpsData->fixType >= 3) {
        fixType = GPS_FIX_3D;
    } else if (gpsData->fixType >= 2) {
        fixType = GPS_FIX_2D;
    }

    const float northMps = gpsData->velocity.x;
    const float eastMps = gpsData->velocity.y;
    const float downMps = gpsData->velocity.z;
    const float groundSpeed = sqrtf(sq(northMps) + sq(eastMps));
    int16_t groundCourse = 0;

    if (groundSpeed > 0.05f) {
        groundCourse = constrainToInt16(RADIANS_TO_DECIDEGREES(atan2f(eastMps, northMps)));
        groundCourse = pasWrap3600(groundCourse);
    }

    gpsFakeSet(
        fixType,
        16,
        (int32_t)lroundf(gpsData->latitude * 10000000.0f),
        (int32_t)lroundf(gpsData->longitude * 10000000.0f),
        (int32_t)lroundf(gpsData->altitude * 100.0f),
        (int16_t)lroundf(groundSpeed * 100.0f),
        groundCourse,
        (int16_t)lroundf(northMps * 100.0f),
        (int16_t)lroundf(eastMps * 100.0f),
        (int16_t)lroundf(downMps * 100.0f),
        0
    );
}

static void pasApplyBarometerData(const pasBarometerData_t *barometerData)
{
    fakeBaroSet(
        (int32_t)lroundf(barometerData->pressure),
        DEGREES_TO_CENTIDEGREES(PAS_DEFAULT_TEMP_C)
    );
}

static void pasApplyMagnetometerData(const pasMagnetometerData_t *magnetometerData)
{
    if (pasCtx.useImu) {
        fakeMagSet(
            constrainToInt16(magnetometerData->magneticFieldBody.x * 1024.0f),
            constrainToInt16(-magnetometerData->magneticFieldBody.y * 1024.0f),
            constrainToInt16(-magnetometerData->magneticFieldBody.z * 1024.0f)
        );
    }
}

static void pasSetZeroOutputs(pasContext_t *ctx)
{
    const float zeroOutputs[PAS_CONTROL_OUTPUT_COUNT] = {0};
    pasSetControlSignals(ctx->socket, ctx->robotPath, zeroOutputs);
}

static void pasMaybeResetOrientationOnDisarm(pasContext_t *ctx, const pasKinematicsData_t *kinematicsData, const pasImuData_t *imuData)
{
    const bool armed = ARMING_FLAG(ARMED);
    if (armed || !ctx->prevArmed) {
        ctx->prevArmed = armed;
        return;
    }

    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;
    pasQuaternionToInavAttitude(&imuData->orientation, &roll, &pitch, &yaw);

    const pasQuaternion_t levelOrientation = {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
    };

    pasSetZeroOutputs(ctx);
    if (!pasSetPose(
            ctx->socket,
            ctx->robotPath,
            kinematicsData->pose.position.x,
            kinematicsData->pose.position.y,
            kinematicsData->pose.position.z,
            &levelOrientation)) {
        ctx->prevArmed = armed;
        return;
    }

    ctx->havePrevAttitude = false;
    fprintf(stderr,
        "[PAS] auto_reset_orientation_on_disarm=1 x=%.3f y=%.3f z=%.3f roll=%.1f pitch=%.1f yaw=%.1f\n",
        kinematicsData->pose.position.x,
        kinematicsData->pose.position.y,
        kinematicsData->pose.position.z,
        roll / 10.0f,
        pitch / 10.0f,
        yaw / 10.0f
    );

    ctx->prevArmed = armed;
}

static void pasCollectMotorOutputs(float motorOutputs[PAS_CONTROL_OUTPUT_COUNT])
{
    for (int i = 0; i < PAS_CONTROL_OUTPUT_COUNT; i++) {
        motorOutputs[i] = constrainf(PWM_TO_FLOAT_0_1(motor[i]), 0.0f, 1.0f);
    }
}

static void *pasWorker(void *arg)
{
    pasContext_t *ctx = (pasContext_t *)arg;

    while (ctx->running) {
        float motorOutputs[PAS_CONTROL_OUTPUT_COUNT];
        pasCollectMotorOutputs(motorOutputs);

        if (!pasSetControlSignals(ctx->socket, ctx->robotPath, motorOutputs)) {
            delayMicroseconds(PAS_FAILURE_DELAY_US);
            continue;
        }

        int64_t simTimeNs = 0;
        if (!pasContinueForNSteps(ctx->socket, ctx->worldPath, 1, &simTimeNs)) {
            delayMicroseconds(PAS_FAILURE_DELAY_US);
            continue;
        }

        if (!pasWaitForTopicData(ctx, simTimeNs)) {
            delayMicroseconds(PAS_FAILURE_DELAY_US);
            continue;
        }

        pasKinematicsData_t kinematicsData;
        if (!pasFetchGroundTruthKinematics(ctx->socket, ctx->robotPath, &kinematicsData)) {
            delayMicroseconds(PAS_FAILURE_DELAY_US);
            continue;
        }

        ctx->lastGtLinearAccelerationWorld = kinematicsData.linearAccelerationWorld;

        pasApplyImuData(ctx, &ctx->imuData, &kinematicsData.pose);
        pasApplyGpsData(&ctx->gpsData);
        pasApplyBarometerData(&ctx->barometerData);
        if (ctx->useImu) {
            pasApplyMagnetometerData(&ctx->magnetometerData);
        }
        fakeBattSensorSetVbat(PAS_DEFAULT_VBAT_CENTIVOLTS);
        pasMaybeResetOrientationOnDisarm(ctx, &kinematicsData, &ctx->imuData);

        if (!ctx->initialized) {
            ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
            ENABLE_STATE(ACCELEROMETER_CALIBRATED);
            ctx->initialized = true;

            fprintf(stderr,
                "[PAS] sim_time_ns=%" PRId64 " imu_time_ns=%" PRId64 " gt_time_ns=%" PRId64 " pos_ned=%.2f,%.2f,%.2f roll=%.1f pitch=%.1f yaw=%.1f lat=%.7f lon=%.7f alt_m=%.2f gps_course_deg=%.1f pressure_pa=%.2f fix_type=%d mag_body=%.3f,%.3f,%.3f motor_rr=%.3f motor_fr=%.3f motor_rl=%.3f motor_fl=%.3f\n",
                simTimeNs,
                ctx->imuData.timeStamp,
                kinematicsData.timeStamp,
                kinematicsData.pose.position.x,
                kinematicsData.pose.position.y,
                kinematicsData.pose.position.z,
                ctx->prevRoll / 10.0f,
                ctx->prevPitch / 10.0f,
                ctx->prevYaw / 10.0f,
                ctx->gpsData.latitude,
                ctx->gpsData.longitude,
                ctx->gpsData.altitude,
                atan2f(ctx->gpsData.velocity.y, ctx->gpsData.velocity.x) * (180.0f / M_PIf),
                ctx->barometerData.pressure,
                ctx->gpsData.fixType,
                ctx->magnetometerData.magneticFieldBody.x,
                ctx->magnetometerData.magneticFieldBody.y,
                ctx->magnetometerData.magneticFieldBody.z,
                motorOutputs[0],
                motorOutputs[1],
                motorOutputs[2],
                motorOutputs[3]
            );
        }

        ctx->loopCount++;
        if ((ctx->loopCount % PAS_LOG_PERIOD) == 0) {
            fprintf(stderr,
                "[PAS] loop=%" PRIu64 " sim_time_ns=%" PRId64 " imu_time_ns=%" PRId64 " gt_time_ns=%" PRId64 " pos_ned=%.2f,%.2f,%.2f roll=%.1f pitch=%.1f yaw=%.1f alt_m=%.2f gps_course_deg=%.1f eph=%.2f epv=%.2f gt_acc_world=%.2f,%.2f,%.2f imu_acc=%.2f,%.2f,%.2f fake_acc=%.0f,%.0f,%.0f mag_body=%.3f,%.3f,%.3f motor_rr=%.3f motor_fr=%.3f motor_rl=%.3f motor_fl=%.3f\n",
                ctx->loopCount,
                simTimeNs,
                ctx->imuData.timeStamp,
                kinematicsData.timeStamp,
                kinematicsData.pose.position.x,
                kinematicsData.pose.position.y,
                kinematicsData.pose.position.z,
                ctx->prevRoll / 10.0f,
                ctx->prevPitch / 10.0f,
                ctx->prevYaw / 10.0f,
                ctx->gpsData.altitude,
                atan2f(ctx->gpsData.velocity.y, ctx->gpsData.velocity.x) * (180.0f / M_PIf),
                ctx->gpsData.eph,
                ctx->gpsData.epv,
                ctx->lastGtLinearAccelerationWorld.x,
                ctx->lastGtLinearAccelerationWorld.y,
                ctx->lastGtLinearAccelerationWorld.z,
                ctx->lastImuLinearAcceleration.x,
                ctx->lastImuLinearAcceleration.y,
                ctx->lastImuLinearAcceleration.z,
                ctx->lastFakeAcc.x,
                ctx->lastFakeAcc.y,
                ctx->lastFakeAcc.z,
                ctx->magnetometerData.magneticFieldBody.x,
                ctx->magnetometerData.magneticFieldBody.y,
                ctx->magnetometerData.magneticFieldBody.z,
                motorOutputs[0],
                motorOutputs[1],
                motorOutputs[2],
                motorOutputs[3]
            );
        }

        unlockMainPID();
    }

    pasSetZeroOutputs(ctx);
    return NULL;
}

bool simProjectAirSimInit(char *ip, int port, bool imu, bool fastMode)
{
    memset(&pasCtx, 0, sizeof(pasCtx));
    pasCtx.useImu = imu;

    char *sceneConfig = NULL;
    size_t sceneConfigLength = 0;
    if (!pasReadSceneConfig(&sceneConfig, &sceneConfigLength)) {
        return false;
    }

    if (fastMode && !pasApplyFastSceneConfig(sceneConfig)) {
        free(sceneConfig);
        return false;
    }

    int socketResult = nng_req0_open(&pasCtx.socket);
    if (socketResult != 0) {
        fprintf(stderr, "[PAS] req0_open_error=%d req0_open_error_text=%s\n", socketResult, nng_strerror(socketResult));
        free(sceneConfig);
        return false;
    }

    int timeoutResult = nng_socket_set_ms(pasCtx.socket, NNG_OPT_SENDTIMEO, PAS_SEND_TIMEOUT_MS);
    if (timeoutResult == 0) {
        timeoutResult = nng_socket_set_ms(pasCtx.socket, NNG_OPT_RECVTIMEO, PAS_RECV_TIMEOUT_MS);
    }
    if (timeoutResult != 0) {
        fprintf(stderr, "[PAS] socket_timeout_error=%d socket_timeout_error_text=%s\n", timeoutResult, nng_strerror(timeoutResult));
        nng_close(pasCtx.socket);
        free(sceneConfig);
        return false;
    }

    const int servicePort = (port > 0) ? port : PAS_DEFAULT_SERVICE_PORT;
    const int topicPort = (port > 0) ? (port - 1) : PAS_DEFAULT_TOPIC_PORT;
    snprintf(pasCtx.serviceUrl, sizeof(pasCtx.serviceUrl), "tcp://%s:%d", ip, servicePort);
    snprintf(pasCtx.topicUrl, sizeof(pasCtx.topicUrl), "tcp://%s:%d", ip, topicPort);

    const int dialResult = nng_dial(pasCtx.socket, pasCtx.serviceUrl, NULL, 0);
    if (dialResult != 0) {
        fprintf(stderr, "[PAS] service_url=%s nng_dial_error=%d nng_dial_error_text=%s\n", pasCtx.serviceUrl, dialResult, nng_strerror(dialResult));
        nng_close(pasCtx.socket);
        free(sceneConfig);
        return false;
    }

    if (!pasLoadScene(pasCtx.socket, sceneConfig, pasCtx.sceneId, sizeof(pasCtx.sceneId))) {
        nng_close(pasCtx.socket);
        free(sceneConfig);
        return false;
    }

    free(sceneConfig);

    snprintf(pasCtx.worldPath, sizeof(pasCtx.worldPath), "/Sim/%s", pasCtx.sceneId);
    snprintf(pasCtx.robotPath, sizeof(pasCtx.robotPath), "%s/robots/%s", pasCtx.worldPath, PAS_SCENE_ROBOT_NAME);
    snprintf(pasCtx.imuPath, sizeof(pasCtx.imuPath), "%s/sensors/%s/imu_kinematics", pasCtx.robotPath, PAS_SENSOR_IMU);
    snprintf(pasCtx.actualPosePath, sizeof(pasCtx.actualPosePath), "%s/actual_pose", pasCtx.robotPath);
    snprintf(pasCtx.gpsPath, sizeof(pasCtx.gpsPath), "%s/sensors/%s/gps", pasCtx.robotPath, PAS_SENSOR_GPS);
    snprintf(pasCtx.barometerPath, sizeof(pasCtx.barometerPath), "%s/sensors/%s/barometer", pasCtx.robotPath, PAS_SENSOR_BAROMETER);
    snprintf(pasCtx.magnetometerPath, sizeof(pasCtx.magnetometerPath), "%s/sensors/%s/magnetometer", pasCtx.robotPath, PAS_SENSOR_MAGNETOMETER);

    socketResult = nng_pair0_open(&pasCtx.topicSocket);
    if (socketResult != 0) {
        fprintf(stderr, "[PAS] topic_pair0_open_error=%d topic_pair0_open_error_text=%s\n", socketResult, nng_strerror(socketResult));
        nng_close(pasCtx.socket);
        return false;
    }

    timeoutResult = nng_socket_set_ms(pasCtx.topicSocket, NNG_OPT_SENDTIMEO, PAS_SEND_TIMEOUT_MS);
    if (timeoutResult != 0) {
        fprintf(stderr, "[PAS] topic_socket_timeout_error=%d topic_socket_timeout_error_text=%s\n", timeoutResult, nng_strerror(timeoutResult));
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    const int topicDialResult = nng_dial(pasCtx.topicSocket, pasCtx.topicUrl, NULL, 0);
    if (topicDialResult != 0) {
        fprintf(stderr, "[PAS] topic_url=%s topic_dial_error=%d topic_dial_error_text=%s\n", pasCtx.topicUrl, topicDialResult, nng_strerror(topicDialResult));
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    if (!pasSubscribeTopics(&pasCtx)) {
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    if (!pasGetGroundZ(pasCtx.socket, pasCtx.worldPath, 0.0f, 0.0f, &pasCtx.groundZ)) {
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    const pasQuaternion_t levelOrientation = {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
    };

    const float snappedZ = pasCtx.groundZ - PAS_FRAME_HALF_HEIGHT_M;
    if (!pasSetPose(pasCtx.socket, pasCtx.robotPath, 0.0f, 0.0f, snappedZ, &levelOrientation)) {
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    pasSetZeroOutputs(&pasCtx);

    int64_t simTimeNs = 0;
    if (!pasContinueForNSteps(pasCtx.socket, pasCtx.worldPath, 1, &simTimeNs)) {
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    pasKinematicsData_t initKinematicsData;
    if (!pasFetchImuData(pasCtx.socket, pasCtx.imuPath, &pasCtx.imuData) ||
        !pasFetchGroundTruthKinematics(pasCtx.socket, pasCtx.robotPath, &initKinematicsData) ||
        !pasFetchGpsData(pasCtx.socket, pasCtx.gpsPath, &pasCtx.gpsData) ||
        !pasFetchBarometerData(pasCtx.socket, pasCtx.barometerPath, &pasCtx.barometerData) ||
        (pasCtx.useImu && !pasFetchMagnetometerData(pasCtx.socket, pasCtx.magnetometerPath, &pasCtx.magnetometerData))) {
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }
    pasCtx.haveImuData = true;
    pasCtx.actualPoseData.timeStamp = initKinematicsData.timeStamp;
    pasCtx.actualPoseData.pose = initKinematicsData.pose;
    pasCtx.lastGtLinearAccelerationWorld = initKinematicsData.linearAccelerationWorld;
    pasCtx.haveActualPoseData = true;
    pasCtx.haveGpsData = true;
    pasCtx.haveBarometerData = true;
    pasCtx.haveMagnetometerData = pasCtx.useImu;

    fprintf(stderr, "[PAS] service_url=%s topic_url=%s scene_id=%s world_path=%s fast_mode=%d step_ns=%s ground_z=%.3f snap_z=%.3f sim_time_ns=%" PRId64 "\n",
        pasCtx.serviceUrl, pasCtx.topicUrl, pasCtx.sceneId, pasCtx.worldPath, fastMode, fastMode ? PAS_FAST_STEP_NS_TEXT : "3000000", pasCtx.groundZ, snappedZ, simTimeNs);

    pasCtx.running = true;
    const int threadResult = pthread_create(&pasCtx.workerThread, NULL, pasWorker, &pasCtx);
    if (threadResult != 0) {
        fprintf(stderr, "[PAS] pthread_create_error=%d\n", threadResult);
        pasCtx.running = false;
        nng_close(pasCtx.topicSocket);
        nng_close(pasCtx.socket);
        return false;
    }

    const uint32_t waitStartMs = millis();
    while (!pasCtx.initialized) {
        if ((millis() - waitStartMs) > PAS_INIT_WAIT_MS) {
            fprintf(stderr, "[PAS] init_timeout_ms=%d service_url=%s scene_id=%s\n", PAS_INIT_WAIT_MS, pasCtx.serviceUrl, pasCtx.sceneId);
            pasCtx.running = false;
            nng_close(pasCtx.topicSocket);
            nng_close(pasCtx.socket);
            return false;
        }

        delay(50);
    }

    return true;
}
