/** ********************************************************************
*
* @mainpage NarPod API
*
* Copyright (c) 2010-2024  Nators
*
* File name: NarPodControl.h
*
* This is the software interface to the Nators NarPod system.
* Please refer to the Programmer's Guide for a detailed documentation.
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  NATORS  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
********************************************************************* */

#ifndef NARPOD_H
#define NARPOD_H


#ifdef NARPOD_EXPORTS
#define NARPOD_API __declspec(dllexport)
#else
#define NARPOD_API __declspec(dllimport)
#endif

#define NARPOD_CC  __cdecl
    /************************************************************************
    **                          API version number                         **
    *************************************************************************/

    /** @constants ApiVersion */
#define NARPOD_API_VERSION_MAJOR                   1
#define NARPOD_API_VERSION_MINOR                   0
#define NARPOD_API_VERSION_UPDATE                  12

    typedef unsigned int NARPOD_STATUS;
    typedef unsigned int NARPOD_INDEX;
    typedef unsigned int NARPOD_PACKET_TYPE;
    /** @endconstants ApiVersion */
    typedef struct Narpod_packet {
        NARPOD_PACKET_TYPE packetType; // type of packet
        unsigned char data0; // data field
        signed int data1; // data field
        signed int data2; // data field
        signed int data3; // data field
        signed int data4; // data field
        signed int data5; // data field
        signed int data6; // data field
    } NARPOD_PACKET;

    /************************************************************************
    **                            status codes                             **
    *************************************************************************/
    /** @constants ErrorCode */

#define NARPOD_OK                                  0//ok
#define NARPOD_OTHER_ERROR                         1//其他错误
#define NARPOD_SYSTEM_NOT_INITIALIZED_ERROR        2//未初始化系统
#define NARPOD_NO_SYSTEMS_FOUND_ERROR              3//未找到系统
#define NARPOD_INVALID_PARAMETER_ERROR             4//无效参数
#define NARPOD_COMMUNICATION_ERROR                 5//通信错误
#define NARPOD_UNKNOWN_PROPERTY_ERROR              6//未知属性错误
#define NARPOD_RESOURCE_TOO_OLD_ERROR              7//资源过期
#define NARPOD_FEATURE_UNAVAILABLE_ERROR           8//功能不可用
#define NARPOD_INVALID_SYSTEM_LOCATOR_ERROR        9//无效系统定位器
#define NARPOD_QUERYBUFFER_SIZE_ERROR              10//缓冲区大小错误
#define NARPOD_COMMUNICATION_TIMEOUT_ERROR         11//通信超时
#define NARPOD_DRIVER_ERROR                        12//驱动错误
#define NARPOD_READ_ERROR                          13//读错误-
#define NARPOD_WRITE_ERROR                         14//写错误-
#define NARPOD_INTERNAL_ERROR                      15//未知错误-
#define NARPOD_TRANSMIT_ERROR                      16//传输错误-
#define NARPOD_TOO_MANY_SYSTEMS_ERROR              17//系统过多-
#define NARPOD_STATUS_CODE_UNKNOWN_ERROR           500//未知状态码
#define NARPOD_INVALID_ID_ERROR                    501//无效id
#define NARPOD_INITIALIZED_ERROR                   502//初始化错误
#define NARPOD_HARDWARE_MODEL_UNKNOWN_ERROR        503//硬件型号未知
#define NARPOD_WRONG_COMM_MODE_ERROR               504//通信模式错误
#define NARPOD_NOT_INITIALIZED_ERROR               505//未初始化
#define NARPOD_INVALID_SYSTEM_ID_ERROR             506//无效系统id
#define NARPOD_NOT_ENOUGH_CHANNELS_ERROR           507//无足够通道
#define NARPOD_INVALID_CHANNEL_ERROR               508//无效通道
#define NARPOD_CHANNEL_USED_ERROR                  509//通道被使用
#define NARPOD_SENSORS_DISABLED_ERROR              510//传感器禁用
#define NARPOD_WRONG_SENSOR_TYPE_ERROR             511//传感器类型错误
#define NARPOD_SYSTEM_CONFIGURATION_ERROR          512//系统配置错误
#define NARPOD_SENSOR_NOT_FOUND_ERROR              513//未找到传感器
#define NARPOD_STOPPED_ERROR                       514//停止
#define NARPOD_BUSY_ERROR                          515//忙
#define NARPOD_NOT_REFERENCED_ERROR                550//没有参考
#define NARPOD_POSE_UNREACHABLE_ERROR              551//不能达到
#define NARPOD_COMMAND_OVERRIDDEN_ERROR            552//指令覆盖
#define NARPOD_ENDSTOP_REACHED_ERROR               553//到达终点
#define NARPOD_NOT_STOPPED_ERROR                   554//不能停止
#define NARPOD_COULD_NOT_REFERENCE_ERROR           555//不能寻零
#define NARPOD_COULD_NOT_CALIBRATE_ERROR           556//不能校准



/** @endconstants ErrorCode */
/************************************************************************
**                             constants                               **
************************************************************************/

/**Narpod Property for Narpod_Set_Property**/
#define FREF_METHOD					0x01
#define PIVOT_MODE					0x06
#define HardwareModel				0x07

/**Narpod Value for Narpod_Set_Property**/
#define METHOD_DEFAULT				0x00
#define METHOD_CHANNEL1				0x01
#define METHOD_CHANNEL2				0x02
#define METHOD_CHANNEL3				0x03
#define METHOD_CHANNEL4				0x04
#define METHOD_CHANNEL5				0x05
#define METHOD_CHANNEL6				0x06
/**Narpod Pivot Mode Property Value**/
#define PIVOT_RELATIVE				0x00
#define PIVOT_FIXED					0x01

/**NarPod HardwareModel Property Value**/
#define MODEL_HEXAPOD110			0x00
#define MODEL_HEXAPOD70			    0x01
#define MODEL_HEXAPOD70U			0x02
#define MODEL_TRIPOD70			    0x10

/**NarPod value Narpod_IsReferencede/Narpod_IsPoseReachable**/
#define NARPOD_FALSE	            0x00
#define NARPOD_TRUE	                0x01

/** Narpod_GetMoveStatus */
#define NARPOD_STOP                                0
#define NARPOD_HOLDING                             3
#define NARPOD_MOVING                              4
#define NARPOD_REFERENCING                         7
#define NARPOD_CALIBRATING                         8

/** Narpod_SetPIDParameter */
#define CHANNEL_PX                   0X01
#define CHANNEL_PY                   0X02
#define CHANNEL_PZ                   0X03
#define CHANNEL_RX                   0X04
#define CHANNEL_RY                   0X05
#define CHANNEL_RZ                   0X06

#ifdef __cplusplus
    extern "C" {
#endif
/*********************************************************************************************************************************/
    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetDLLVersion(unsigned int* major, unsigned int* minor, unsigned int* update);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Open(unsigned int* ntpodId, const char* locator, const char* options);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Close(unsigned int ntpodId);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_FindSystems(const char* options, char* systems, unsigned int* ioBufferSize);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetSystemLocator(unsigned int ntpodId, char* locator, unsigned int* ioBufferSize);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetSystemLocator(unsigned int ntpodId, char* inBuffer);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Set_Property(unsigned int ntpodId, unsigned int property, unsigned int value);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Get_Property(unsigned int ntpodId, unsigned int property, unsigned int* value);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Calibrate(unsigned int ntpodId);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_FindReferenceMarks(unsigned int ntpodId);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_IsReferenced(unsigned int ntpodId, int* referenced);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetSensorMode(unsigned int ntpodId, unsigned int mode);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetSensorMode(unsigned int ntpodId, unsigned int* mode);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetMaxFrequency(unsigned int ntpodId, unsigned int frequency);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetMaxFrequency(unsigned int ntpodId, unsigned int* frequency);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetSpeed(unsigned int ntpodId, unsigned int speed);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetSpeed(unsigned int ntpodId, unsigned int* speed);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetPivot(unsigned int ntpodId, signed int pivotx, signed int pivoty, signed int pivotz);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetPivot(unsigned int ntpodId, signed int* pivotx, signed int* pivoty, signed int* pivotz);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_IsPoseReachable(unsigned int ntpodId, signed int positionx, signed int positiony, signed int positionz, signed int rotationx, signed int rotationy, signed int rotationz, unsigned int* reachable); 

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetMoveStatus(unsigned int ntpodId, unsigned int* status);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Move(unsigned int ntpodId, signed int positionx, signed int positiony, signed int positionz, signed int rotationx, signed int rotationy, signed int rotationz);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Stop(unsigned int ntpodId);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetPosition(unsigned int ntpodId, signed int* positionx, signed int* positiony, signed int* positionz, signed int* rotationx, signed int* rotationy, signed int* rotationz);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Clear(unsigned int ntpodId);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_Move_Absolute_Indep(unsigned int ntpodId, signed int positionx, signed int positiony, signed int positionz, signed int rotationx, signed int rotationy, signed int rotationz);

    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetCoordinateSystem(unsigned int ntpodId, signed int positionx, signed int positiony, signed int positionz, signed int rotationx, signed int rotationy, signed int rotationz);
    
    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_GetCoordinateSystem(unsigned int ntpodId, signed int* positionx, signed int* positiony, signed int* positionz, signed int* rotationx, signed int* rotationy, signed int* rotationz);
    
    NARPOD_API
        NARPOD_STATUS NARPOD_CC Narpod_SetPIDParameter(unsigned int ntpodId, unsigned int channel, float Proportion, float Integral, float Derivative);
    /********************************************************************************************************************************/






#ifdef __cplusplus
}
#endif 

#endif
