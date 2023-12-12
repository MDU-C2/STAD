#ifndef CameraInterface
#define CameraInterface

#include <framework/DriveWorksSample.hpp>
#include <dw/Driveworks.h>  

//holds all available camera instances of one port and can return the frames from it
class cameraInterface
{
    public:
    //empty constructor
    cameraInterface(){}

    //constructor that creates and starts a camera decided by params input
    cameraInterface(const dwContextHandle_t context, const dwSALHandle_t sal, const dwSensorParams params, const int camerasOnPort);
    void readImages(dwCameraFrameHandle_t* frames);

    //function returns properties of the camera sensor
    void getCameraProperties(dwCameraProperties *properties)
    {
        if(m_camera)
        {
            dwSensorCamera_getSensorProperties(properties, m_camera);
        }
    }

    void releaseHandle()
    {   
        if(m_camera)
        {
            dwSAL_releaseSensor(&m_camera);
        }
    }

    dwSensorHandle_t getSensorHandle()
    {
        return m_camera;
    }

    private:
    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwSALHandle_t m_sal = DW_NULL_HANDLE;
    dwSensorHandle_t m_camera = DW_NULL_HANDLE;
    int m_camerasOnPort = 0;
};


cameraInterface::cameraInterface(const dwContextHandle_t context, const dwSALHandle_t sal, const dwSensorParams params, const int camerasOnPort)
{
    m_camerasOnPort = camerasOnPort;
    m_context = context;
    m_sal = sal;

    if(camerasOnPort)
    {
        CHECK_DW_ERROR(dwSAL_createSensor(&m_camera, params, m_sal));
        CHECK_DW_ERROR(dwSensor_start(m_camera));

        dwStatus status = DW_NOT_READY;
        dwCameraFrameHandle_t frame;
        //wait until camera does not return DW_NOT_READY
        while (status == DW_NOT_READY) 
        {
            status = dwSensorCamera_readFrame(&frame, 0, 66000, m_camera);
        }
        dwSensorCamera_returnFrame(&frame);
    }
}


void cameraInterface::readImages(dwCameraFrameHandle_t* frames)
{
    for(int port = 0; port < m_camerasOnPort; port++)
    {
        if(frames[port])
        {
            dwSensorCamera_returnFrame(&frames[port]);
        }
        //try to read frame until the frame is ready
        dwStatus status = DW_NOT_READY;
        do {
            status = dwSensorCamera_readFrame(&frames[port], port, 1000000, m_camera);
        } while (status == DW_NOT_READY);

        //if end of stream set NULL handle
        if(status == DW_END_OF_STREAM)
        {
            std::cout << "Camera reached end of stream." << std::endl;
            frames[port] = DW_NULL_HANDLE;
        }
        //else if the status is some failure throw an error
        else if(status != DW_SUCCESS)
        {
            throw std::runtime_error("Error reading from camera");
        }
    }
}

#endif