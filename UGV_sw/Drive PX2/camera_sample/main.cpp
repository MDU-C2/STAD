/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////

#include <dw/core/VersionCurrent.h>
#include <framework/DriveWorksSample.hpp>
#include <math.h>


// Include all relevant DriveWorks modules
#include <dw/Driveworks.h>

#include <src/camera_sample/CameraInterface.hpp>

using namespace dw_samples::common;


//------------------------------------------------------------------------------
// Camera sample that utilizes the driveworkssample class for graphics etc.
// The Program can be expanded to read multiple cameras and later add object detection
//------------------------------------------------------------------------------
class CameraSample : public DriveWorksSample
{
public:
    CameraSample(const ProgramArguments& args)
        : DriveWorksSample(args), m_recordCameras(args.enabled("record")) {}

    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks(dwContextHandle_t& context) const
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};
        sdkParams.dataPath = DataPath::get_cstr();

        sdkParams.eglDisplay = getEGLDisplay();

        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    }

    /// -----------------------------
    /// Initialize everything of a sample here incl. SDK components
    /// -----------------------------
    bool onInitialize() override
    {
        log("Starting my sample application...\n");

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        // -----------------------------
        // Initialize RendererEngine for rendering in GL and spliting window
        // -----------------------------
        {
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            params.defaultTile.lineWidth = 2.0f;
            params.maxBufferCount = 1;
            params.bounds = {0, 0};
            params.bounds.width = static_cast<float32_t>(getWindowWidth());
            params.bounds.height = static_cast<float32_t>(getWindowHeight());

            uint32_t tilesPerRow = std::ceil(m_totalCameras/2);
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderer, &params, m_context));

            dwRenderEngineTileState paramList[8];
            for (uint32_t i = 0; i < m_totalCameras; ++i) 
            {
                dwRenderEngine_initTileState(&paramList[i]);
                paramList[i].modelViewMatrix = DW_IDENTITY_TRANSFORMATION;
            }

            dwRenderEngine_addTilesByCount(m_renderTile, m_totalCameras, tilesPerRow, paramList, m_renderer);
        }
        // -----------------------------
        // Initialize Camera
        // -----------------------------  
        {
            std::string cameraMaskA = "";
            std::string cameraMaskB = "";
            int camerasOnPortA = std::min(m_totalCameras, 4);
            int camerasOnPortB = std::max(m_totalCameras - 4, 0);
            for(int i = 1; i <= 8; i++)
            {
                if(m_totalCameras >= i)
                {
                    if(i <= 4) cameraMaskA = "1" + cameraMaskA;
                    else cameraMaskB = "1" + cameraMaskB;
                }
                else
                {
                    if(i <= 4) cameraMaskA = "0" + cameraMaskA;
                    else cameraMaskB = "0" + cameraMaskB;
                }
            }

            dwSensorParams params = {};
            std::string parameterString = "output-format=yuv,fifo-size=3";
            parameterString += ",camera-type=ar0231-rccb-bae-sf3324"; //the camera type we have
            parameterString += ",serialize=true";
            parameterString += ",camera-count=4";//makes all of the csi-ports available
            params.protocol = "camera.gmsl"; //we are using gmsl cameras

            std::string paramA = parameterString + ",csi-port=a";//the csi port, can be A -> a, B -> c or C -> e
            paramA += ",camera-mask=" + cameraMaskA;//selector mask for which camera in the csi-port. 0001 is port 0, 0010 is port 1 etc so this becomes port A1
            params.parameters = paramA.c_str();

            m_camera[0] = cameraInterface(m_context, m_sal, params, camerasOnPortA); //create a camera instance that can return images

            std::string paramB = parameterString + ",csi-port=c";// is port B, i think it is that tegra B has the ports named b, d, e (not sure just a guess)
            paramB += ",camera-mask=" + cameraMaskB;//selector mask for which camera in the csi-port. 0001 is port 0, 0010 is port 1 etc so this becomes port A1
            params.parameters = paramB.c_str();

            m_camera[1] = cameraInterface(m_context, m_sal, params, camerasOnPortB);
        }

        //------------------------------------------------------------------------------
        // initializes streamer from cuda to GL image to be able to render it in GL
        // -----------------------------------------
        {
            dwCameraProperties camProp{};
            m_camera[0].getCameraProperties(&camProp);
            dwImageProperties glImageProperties{};
            glImageProperties.width = camProp.resolution.x;
            glImageProperties.height = camProp.resolution.y;
            glImageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            glImageProperties.type = DW_IMAGE_CUDA;
            CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamerCUDAtoGL, &glImageProperties, DW_IMAGE_GL, m_context));
        }

        //------------------------------------------
        // initializes serializer for recording camera
        // -----------------------------------------
        {
            if (m_recordCameras) 
            {
                for(int i = 0; i < m_totalCameras; i++)
                {
                    dwSerializerParams serializerParams;
                    std::string newParams = "";
                    newParams += "format=h264";
                    newParams += ",bitrate=8000000";
                    newParams += ",framerate=30";
                    newParams += ",type=disk,file=camera" + std::to_string(i) +".h264";
                    newParams += ",slave=0";

                    serializerParams.parameters = newParams.c_str();
                    serializerParams.onData     = nullptr;

                    if(i <= 3) 
                    {
                        CHECK_DW_ERROR(dwSensorSerializer_initialize(&m_serializer[i], &serializerParams, m_camera[0].getSensorHandle()));
                    }
                    else 
                    {
                        CHECK_DW_ERROR(dwSensorSerializer_initialize(&m_serializer[i], &serializerParams, m_camera[1].getSensorHandle()));
                    }
                    CHECK_DW_ERROR(dwSensorSerializer_start(m_serializer[i]));
                }
            }
        }
        return true;
    }

    ///------------------------------------------------------------------------------
    /// This method is executed on release, free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if(m_recordCameras)
        {
            for(int i = 0; i < m_totalCameras; i++) 
            {
                dwSensorSerializer_release(&m_serializer[i]);
            }            
        }
        dwRenderEngine_release(&m_renderer);

        m_camera[0].releaseHandle();
        m_camera[1].releaseHandle();

        dwSAL_release(&m_sal);

        dwRelease(&m_context);

        dwLogger_release();
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRenderEngine_reset(m_renderer);
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderEngine_setBounds(rect, m_renderer);
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - this method is executed for window and console based applications
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        //read camera frame using cameraInterface class
        m_camera[0].readImages(frames);
        m_camera[1].readImages(frames+4);

        //convert cameraFrame to imagehandle
        for(int i = 0; i < m_totalCameras; i++)
        {
            dwSensorCamera_getImage(&framesCUDA[i], DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, frames[i]);
        }
        if (m_recordCameras) 
        {
            for(int i = 0; i < m_totalCameras; i++)
            {
                dwSensorSerializer_serializeCameraFrameAsync(frames[i], m_serializer[i]);
            }
        }

        //do object detection on frame here?
    }

    ///------------------------------------------------------------------------------
    /// Render call of the sample, executed for window based applications only
    ///     - render text on screen
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        for(int i = 0; i < (m_totalCameras); i++)
        {
            dwRenderEngine_setTile(m_renderTile[i], m_renderer);
            dwRenderEngine_resetTile(m_renderer);
            
            // stream that image to the GL domain
            dwImageStreamer_producerSend(framesCUDA[i], m_streamerCUDAtoGL);

            // receive the streamed image as a handle
            dwImageHandle_t frameGL;
            dwImageStreamer_consumerReceive(&frameGL, 132000, m_streamerCUDAtoGL);

            dwImageGL* imageGL = {};
            //get the specific image struct to be able to access texture ID and target
            dwImage_getGL(&imageGL, frameGL);

            // render received texture in specific tile
            dwVector2f range{};
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            dwRenderEngine_setCoordinateRange2D(range, m_renderer);
            dwRenderEngine_renderImage2D(imageGL, {0,0,range.x,range.y}, m_renderer);
            dwRenderEngine_setColor({0,0,0,1}, m_renderer);

            // returned the consumed image
            dwImageStreamer_consumerReturn(&frameGL, m_streamerCUDAtoGL);

            // notify the producer that the work is done
            dwImageStreamer_producerReturn(nullptr, 132000, m_streamerCUDAtoGL);
        }
    }

private:
    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderer = DW_NULL_HANDLE;
    dwSALHandle_t m_sal = DW_NULL_HANDLE;

    static const int m_totalCameras = 2; //change if more than 2 cameras
    cameraInterface m_camera[2]; 
    dwImageHandle_t framesCUDA[8] = {DW_NULL_HANDLE};
    dwCameraFrameHandle_t frames[8] = {DW_NULL_HANDLE};
    dwImageStreamerHandle_t m_streamerCUDAtoGL = DW_NULL_HANDLE;
    uint32_t m_renderTile[8]; //holds the renderable tiles such that each camera gets part of the window for rendering
    dwSensorSerializerHandle_t m_serializer[8];
    bool m_recordCameras;

};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("offscreen", "0"),
                           ProgramArguments::Option_t("record", "0"),
                          },
                          "This is a message shown on console when sample prints help.");

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    CameraSample app(args);

    app.initializeWindow("Camera Example", 1600, 1000, args.enabled("offscreen"));

    return app.run();
}
