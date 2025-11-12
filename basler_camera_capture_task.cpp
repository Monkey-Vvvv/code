#include "basler_camera_capture_task.h"

/**
 * 构造函数
 * 参数: 无
 * 返回: 无
 */
cisdi::BaslerCameraCaptureTask::BaslerCameraCaptureTask()
{
}

/**
 * 析构函数
 * 参数: 无
 * 返回: 无
 */
cisdi::BaslerCameraCaptureTask::~BaslerCameraCaptureTask()
{
}

/**
 * 任务运行主方法
 * 参数: 无
 * 返回: 无
 */
void cisdi::BaslerCameraCaptureTask::run(void *)
{

    spdlog::info("basler相机抓图任务启动.");

    try
    {
        // 设置任务状态为 STARTING
        ApplicationConfig::updateCameraInfoToRedis("cameraCaptureTaskStatus", "STARTING");

        // 从redis读取相机配置
        cameraInfoMap = ApplicationConfig::loadCameraInfoFromRedis();

        // 初始化pylon
        Pylon::PylonInitialize();

        // 定义CBaslerUniversalInstantCamera
        Pylon::CBaslerUniversalInstantCamera camera;

        // 连接和设置相机默认参数
        openAndSetDefaultParam(camera);

        // 声明接收抓图结果数据的智能指针
        Pylon::CGrabResultPtr ptrGrabResult;

        // 定义 CImageFormatConverter
        Pylon::CImageFormatConverter formatConverter;

        // 确定输出像素格式
        formatConverter.OutputPixelFormat = Pylon::PixelType_RGB8packed;

        // 定义 CPylonImage
        Pylon::CPylonImage pylonImage;

        // 开始启动抓取
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // 设置任务状态为 RUNNING
        ApplicationConfig::updateCameraInfoToRedis("cameraCaptureTaskStatus", "RUNNING");

        // 循环获取数据
        int autoExposureNum = 1;
        int newExposureTime = 0;
        static int64_t timenow = 0;
        static int64_t picAll = 0;
        static int64_t picInQueue = 0;
        static int64_t picInfer = 0;
        static int64_t picThrow = 0;
        static int64_t picThrowAll = 0;
        static int64_t picInferThrowAll = 0;
        static int64_t picInferThrow = 0;
        static int64_t picFailed = 0;
        static int64_t picFailedAll = 0;
        static int expose = 0;
        timenow = ApplicationConfig:: getCurrentTimeMs();


        while (camera.IsGrabbing())
        {
            ++picAll;
            // 线程停止判断
            if (!running_flag)
            {
                spdlog::warn("抓图线程停止");
                break;
            }

            // 判断是否更新相机动态参数
            if (ApplicationConfig::updateBaslerCameraDynamicParam)
            {
                spdlog::warn("动态更新相机参数");
                //updateBaslerCameraDynamicParam(camera);
                setCommonParam(camera);
                ApplicationConfig::updateBaslerCameraDynamicParam = false;
            }

            // 等待抓取一张图片并接收，超时时间为5000ms
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // 抓图并生成图片信息
            if (ptrGrabResult->GrabSucceeded())
            {
                try
                {
                    formatConverter.Convert(pylonImage, ptrGrabResult->GetBuffer(), ptrGrabResult->GetBufferSize(),
                                            ptrGrabResult->GetPixelType(), ptrGrabResult->GetWidth(), ptrGrabResult->GetHeight(),
                                            ptrGrabResult->GetPaddingX(), Pylon::ImageOrientation_TopDown);


                    // 使用opencv创建图片Mat
                    cv::Mat imageMat(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3, pylonImage.GetBuffer(), cv::Mat::AUTO_STEP);
                    
                    if(cisdi::ApplicationConfig::cameraCapturePic)
                    {
                        auto capTime = ApplicationConfig:: getCurrentTimeMs();
                        std::string filename = "./capPic/" + std::to_string(capTime)+"_camera.jpg";
                        cv::imwrite(filename.c_str(),imageMat);
                        spdlog::info("camera capture {} success", filename);
                        cisdi::ApplicationConfig::cameraCapturePic = false;
                    }
                    //auto value_before = ApplicationConfig:: getCurrentTimeMs();
                    // 计算调整曝光
                    if (ApplicationConfig::enbaleAutoExposureAlgo)
                    {
                        if ((autoExposureNum % ApplicationConfig::autoExposureAlgoInterval) == 0)
                        {
                            double currentExposureTime = camera.ExposureTimeAbs.GetValue();
                            vector<int> veptinfo;
                            ApplicationConfig::calcExposure.calculate_img_information(imageMat, veptinfo);
                            newExposureTime = ApplicationConfig::calcExposure.calept_binary_overept_faster(veptinfo, currentExposureTime);
                            if (newExposureTime > 0 && newExposureTime != currentExposureTime)
                            {
                                camera.ExposureTimeAbs.SetValue(newExposureTime);
                                //auto redis_value_before = ApplicationConfig:: getCurrentTimeMs();

                                ApplicationConfig::updateCameraInfoToRedis("exposureTime", std::to_string(newExposureTime));
                                //auto redis_value_after = ApplicationConfig:: getCurrentTimeMs();
                                spdlog::info("newExposureTime: {}", newExposureTime);
                                //spdlog::info("newExposureTime: {}, set redis time:{}", newExposureTime,redis_value_after - redis_value_before);
                            }
                            autoExposureNum = 1;
                        }
                        autoExposureNum++;
                    }
                    // else
                    // {
                    //     //spdlog::info("无自动曝光");
                    // }

                    //auto value_after = ApplicationConfig:: getCurrentTimeMs();
                    //spdlog::debug("相机抓图任务抓到图片信息:width:{},height:{},size:{} 曝光计算时间:{}ms", pylonImage.GetWidth(), pylonImage.GetHeight(), pylonImage.GetImageSize(),value_after-value_before);
                    spdlog::debug("相机抓图任务抓到图片信息:width:{},height:{},size:{}", pylonImage.GetWidth(), pylonImage.GetHeight(), pylonImage.GetImageSize());

                    // 将图片传递到图片队列
                    if (ApplicationConfig::baslerImageQueue->size() < 5)
                    {
                        ++picInQueue;
                        ApplicationConfig::baslerImageQueue->push(imageMat.clone());
                    }
                    else
                    {
                        ++picThrow;
                        ++picThrowAll;
                    }

                    if(ApplicationConfig::enbaleInferRtsp)
                    {
                        cv::Mat InferMat = imageMat.clone();
                        if(expose != 0)
                        {
                            cv::putText(InferMat, std::to_string(expose).c_str(), cv::Point(10, pylonImage.GetHeight() - 20), cv::FONT_HERSHEY_SIMPLEX,2, Scalar(255, 255, 255),4,8);
                        }
                        if(expose != newExposureTime && newExposureTime != 0)
                        {
                            expose = newExposureTime;
                        }
                        
                        if(ApplicationConfig::inferImageQueue->size() < 5)
                        {
                            ++picInfer;
                            ApplicationConfig::inferImageQueue->push(InferMat.clone());
                        }
                        else
                        {
                                ++picInferThrow;
                                ++picInferThrowAll;
                        }
                    }
                }
                catch (const Pylon::GenericException &e)
                {
                    spdlog::error("抓图发生异常:{}", e.GetDescription());
                }
            }
            else
            {
                spdlog::error("Pylon: Grab Result Failed! Error: " + std::string(ptrGrabResult->GetErrorDescription()));
                ++picFailed;
                ++picFailedAll;
            }

            auto value = ApplicationConfig:: getCurrentTimeMs();
            if (value - timenow >=5000)
            {
                spdlog::info("=====队列{}=====本次取图数 {}====取图失败 {}====总取图失败 {}====直播图片入队 {}====直播扔图 {}====直播扔图总数 {}====渲染图片入队 {}====渲染扔图 {}====渲染扔图总数 {}=====",
                            ApplicationConfig::baslerImageQueue->size(),picAll,picFailed,picFailedAll,picInQueue,picThrow,picThrowAll,picInfer,picInferThrow,picInferThrowAll);
                
                timenow = value;
                picAll = 0;
                picFailed = 0;
                picInQueue = 0;
                picThrow = 0;
                picInferThrow = 0;
                picInfer = 0;
            }
        }
    }
    catch (const Pylon::GenericException &e)
    {
        spdlog::error("相机抓图任务运行异常:{}", e.what());
    }
    catch (const std::exception &e)
    {
        spdlog::error("相机抓图任务运行异常:{}", e.what());
    }
    catch (...)
    {
        spdlog::error("相机抓图任务运行异常:未知异常");
    }
    // 释放相机资源
    try
    {
        Pylon::PylonTerminate();
    }
    catch (const std::exception &e)
    {
        spdlog::error("释放相机资源异常:{}", e.what());
    }
    catch (...)
    {
        spdlog::error("释放相机资源异常:未知异常");
    }

    // 设置任务状态为 STOPPED
    ApplicationConfig::updateCameraInfoToRedis("cameraCaptureTaskStatus", "STOPPED");

    // 任务结束
    running_flag = false;
    stop_flag = true;
    spdlog::info("相机抓图任务停止.");
}

void cisdi::BaslerCameraCaptureTask::setCommonParam(Pylon::CBaslerUniversalInstantCamera &camera)
{
     // 设置曝光算法配置
    auto commonCameraInfoMap = ApplicationConfig::loadCameraInfoFromRedis();
    std::string autoExposureAlgoStatus = commonCameraInfoMap.find("autoExposureAlgoStatus")->second;
    int autoExposureAlgoInterval = std::stoi(commonCameraInfoMap.find("autoExposureAlgoInterval")->second);
    if (autoExposureAlgoStatus == "OPEN")
    {
        ApplicationConfig::enbaleAutoExposureAlgo = true;
        spdlog::info("开启自动曝光");
    }
    else if (autoExposureAlgoStatus == "CLOSE")
    {
        ApplicationConfig::enbaleAutoExposureAlgo = false;
        spdlog::info("关闭自动曝光");
    }
    else
    {
        spdlog::warn("自动曝光算法状态不支持.");
    }

    if (autoExposureAlgoInterval != 0)
    {
        ApplicationConfig::autoExposureAlgoInterval = autoExposureAlgoInterval;
    }

    // 相机曝光时间
    if (camera.ExposureTimeAbs.IsWritable())
    {
        int exposureTime = std::stoi(commonCameraInfoMap.find("exposureTime")->second);
        camera.ExposureTimeAbs.SetValue(exposureTime);
        spdlog::info("设置相机参数:{},值:{},成功.", "曝光时间", exposureTime);
    }
    // 创建redisTool
    auto redisTool = RedisTool(ApplicationConfig::redisHost, ApplicationConfig::redisPort, ApplicationConfig::redisAuth);
    std::unordered_map<std::string, std::string> AOIControlInfoMap;
    if(redisTool.existsKey("AOIControl"))
    {
	    AOIControlInfoMap = redisTool.readHashValues("AOIControl");
    }

    //设置增益
    if (camera.GainRaw.IsWritable() && AOIControlInfoMap.count("gain") == 1)
    {
	    int cameraGain = std::stoi(AOIControlInfoMap.find("gain")->second);
        int gainMax = camera.GainRaw.GetMax();
        redisTool.writeValue("AOIControl", "gainMax", std::to_string(gainMax));
        if(cameraGain > gainMax)
        {
            spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "GainRaw", cameraGain,gainMax);
            cameraGain = gainMax - 1;
        }
        camera.GainRaw.SetValue(cameraGain);
        spdlog::info("设置相机参数:{},值:{},成功.", "GainRaw", cameraGain);
    }

    //设置X偏移量
    if (camera.OffsetX.IsWritable()&&AOIControlInfoMap.count("offsetX") == 1)
    {
        int cameraOffsetX = std::stoi(AOIControlInfoMap.find("offsetX")->second);
        int OffsetXMax = camera.OffsetX.GetMax();
        OffsetXMax = (OffsetXMax / 10) * 10;
        redisTool.writeValue("AOIControl", "offsetXMax", std::to_string(OffsetXMax));
        if(cameraOffsetX > OffsetXMax)
        {
            spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "OffsetXMax", cameraOffsetX,OffsetXMax);
            cameraOffsetX = OffsetXMax;
        }
        cameraOffsetX = (cameraOffsetX / 10) * 10;
	    camera.OffsetX.SetValue(cameraOffsetX);
        spdlog::info("设置相机参数:{},值:{},成功.", "OffsetX", cameraOffsetX);
    }

    //设置Y偏移量
    if (camera.OffsetY.IsWritable()&&AOIControlInfoMap.count("offsetY") == 1)
    {
        int cameraOffsetY = std::stoi(AOIControlInfoMap.find("offsetY")->second);
        int OffsetYMax = camera.OffsetY.GetMax();
        OffsetYMax = (OffsetYMax / 10) * 10;
        redisTool.writeValue("AOIControl", "offsetYMax", std::to_string(OffsetYMax));
        if(cameraOffsetY > OffsetYMax)
        {
            spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "OffsetYMax", cameraOffsetY,OffsetYMax);
            cameraOffsetY = OffsetYMax - 1;
        }
        cameraOffsetY = (cameraOffsetY / 10) * 10;
        camera.OffsetY.SetValue(cameraOffsetY);
        spdlog::info("设置相机参数:{},值:{},成功.", "OffsetY", cameraOffsetY);
    }

    int WidthtMax = camera.Width.GetMax();
    redisTool.writeValue("camera_info", "widthMax", std::to_string(WidthtMax));
    int HeightMax = camera.Height.GetMax();
    redisTool.writeValue("camera_info", "heightMax", std::to_string(HeightMax));

    //设置gamma模式
    if (AOIControlInfoMap.count("gammaMode") == 1)
    {
	    std::string gammaMode = AOIControlInfoMap.find("gammaMode")->second;
        if(gammaMode == "user")
        {
            camera.GammaSelector.SetValue(Basler_UniversalCameraParams::GammaSelector_User);
        }
        else
        {
            camera.GammaSelector.SetValue(Basler_UniversalCameraParams::GammaSelector_sRGB);
        }
	    spdlog::info("设置相机参数:{},值:{},成功.", "gammaMode", gammaMode);
    }

    //设置gamma
    if (camera.Gamma.IsWritable()&&AOIControlInfoMap.count("gamma") == 1)
    {
	    double cameraGamma = std::stod(AOIControlInfoMap.find("gamma")->second);
        double cameraGammaMax = round(camera.Gamma.GetMax() * 100) / 100  - 0.1;
        if(cameraGammaMax > 0)
        {
            redisTool.writeValue("AOIControl", "gammaMax", std::to_string(cameraGammaMax));
        }
        
        if(cameraGamma - cameraGammaMax > 0.1)
        {
            spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "gamma", cameraGamma,cameraGammaMax);
            cameraGamma = cameraGammaMax - 0.1;
        }

        camera.Gamma.SetValue(cameraGamma);
	    spdlog::info("设置相机参数:{},值:{},成功.", "Gamma", cameraGamma);
    }


}
/**
 * 连接相机并设置默认参数
 * 参数: camera
 * 返回: 无
 */
void cisdi::BaslerCameraCaptureTask::openAndSetDefaultParam(Pylon::CBaslerUniversalInstantCamera &camera)
{
    // 定义CDeviceInfo
    Pylon::CDeviceInfo deviceInfo;

    // 通过ip和端口创建CDeviceInfo实列
    std::string cameraIp = cameraInfoMap.find("ip")->second;
    std::string cameraPort = cameraInfoMap.find("port")->second;
    std::string address = cameraIp + ":" + cameraPort;
    deviceInfo.SetAddress(address.c_str());

    // 通过CDeviceInfo创建相机实列
    camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice(deviceInfo));

    // 开启相机
    camera.Open();

    // 打印当前连接的相机名称
    spdlog::info("已连接到相机IP:{}, 端口:{}, 名称：{}", cameraIp, cameraPort, camera.GetDeviceInfo().GetModelName());

    // 设置相机最大缓冲区,默认为10
    camera.MaxNumBuffer = 5;
    spdlog::info("设置相机参数:{},值:{},成功.", "MaxNumBuffer", "5");
    auto redisTool = RedisTool(ApplicationConfig::redisHost, ApplicationConfig::redisPort, ApplicationConfig::redisAuth);


    // 根据系统配置设置宽度
    int cameraWidth = std::stoi(cameraInfoMap.find("width")->second);
    int WidthtMax = camera.Width.GetMax();
    redisTool.writeValue("camera_info", "widthMax", std::to_string(WidthtMax));
    if(cameraWidth > WidthtMax)
    {
        spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "cameraHeight", cameraWidth,WidthtMax);
        cameraWidth = WidthtMax;
    }
    redisTool.writeValue("camera_info", "width", std::to_string(cameraWidth));
    camera.Width.SetValue(cameraWidth);
    spdlog::info("设置相机参数:{},值:{},成功.", "Width", cameraWidth);



    // 根据系统配置设置高度
    int cameraHeight = std::stoi(cameraInfoMap.find("height")->second);
    int HeightMax = camera.Height.GetMax();
    redisTool.writeValue("camera_info", "heightMax", std::to_string(HeightMax));
    if(cameraHeight > HeightMax)
    {
        spdlog::warn("设置相机参数:{},值:{},超过最大值{}.", "cameraHeight", cameraHeight,HeightMax);
        cameraHeight = HeightMax;
    }
    redisTool.writeValue("camera_info", "height", std::to_string(cameraHeight));
    camera.Height.SetValue(cameraHeight);
    spdlog::info("设置相机参数:{},值:{},成功.", "Height", cameraHeight);

    // 允许将相机的帧采集速率设为指定的值。
    if (camera.AcquisitionFrameRateEnable.IsWritable())
    {
        camera.AcquisitionFrameRateEnable.SetValue(true);
        spdlog::info("设置相机参数:{},值:{},成功.", "帧率控制", true);
    }

    // 相机的帧采集速率（帧/秒）。
    if (camera.AcquisitionFrameRateAbs.IsWritable())
    {
        int cameraFrameRate = std::stoi(cameraInfoMap.find("frameRate")->second);
        camera.AcquisitionFrameRateAbs.SetValue(cameraFrameRate);
        spdlog::info("设置相机参数:{},值:{},成功.", "帧率", cameraFrameRate);
    }

    // 相机PacketSize
    if (camera.GevSCPSPacketSize.IsWritable())
    {
        int cameraPacketSize = std::stoi(cameraInfoMap.find("packetSize")->second);
        camera.GevSCPSPacketSize.SetValue(cameraPacketSize);
        spdlog::info("设置相机参数:{},值:{},成功.", "发包大小", cameraPacketSize);
    }

    // 相机PacketSize
    if (camera.GevSCPD.IsWritable())
    {
        int cameraPacketDelay = std::stoi(cameraInfoMap.find("packetDelay")->second);
        camera.GevSCPD.SetValue(cameraPacketDelay);
        spdlog::info("设置相机参数:{},值:{},成功.", "发包间隔", cameraPacketDelay);
    }
    setCommonParam(camera);
}

/**
 * 更新相机动态参数
 * 参数: camera
 * 返回: 无
 */
void cisdi::BaslerCameraCaptureTask::updateBaslerCameraDynamicParam(Pylon::CBaslerUniversalInstantCamera &camera)
{
    // 重新加载相机信息
    cameraInfoMap = ApplicationConfig::loadCameraInfoFromRedis();

    // 设置相机动态参数 曝光时间
    if (camera.ExposureTimeAbs.IsWritable())
    {
        int exposureTime = std::stoi(cameraInfoMap.find("exposureTime")->second);
        camera.ExposureTimeAbs.SetValue(exposureTime);
        spdlog::info("设置相机动态参数:{},值:{},成功.", "ExposureTimeAbs", exposureTime);
    }

    ApplicationConfig::updateBaslerCameraDynamicParam = false;
}

/**
 * 启动任务方法
 * 参数: 无
 * 返回: 无
 */
void cisdi::BaslerCameraCaptureTask::startTask()
{
    start(true);
}

/**
 * 停止任务方法
 * 参数: 无
 * 返回: 无
 */
void cisdi::BaslerCameraCaptureTask::stopTask()
{
    stop();
}
