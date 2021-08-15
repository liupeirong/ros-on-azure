# Run ROS Cartographer in Azure Functions

This sample shows how to run ROS and Cartographer commands in Azure Functions. It is one way to implement the following use case -

1. A robot moves around a space and generates its lidar data continuously in ros bag files.
1. The bag files are moved to an Azure IoT Edge Blob storage, and from there get automatically uploaded to the cloud.
1. An Azure Function watches the cloud blob storage, and upon the creation of a new blob, runs ROS and/or Cartographer command to process the bag file, for example, compute a map of the robot based on the lidar data.

## Design options

[Azure IoT Hub for ROS](https://github.com/microsoft/ros_azure_iothub) enables an Azure IoT device to subscribe to ROS topics and send the topic messages to Azure IoT Hub. However, Azure IoT Hub has a device-to-cloud message size limit of 256KB, making it an inadequate option for uploading lidar data.

[Azure IoT device to cloud file upload](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-csharp-csharp-file-upload) enables you to upload files from device to cloud using Azure IoT SDK. However, if internet connection to from device to the cloud is not available, upload will fail, and your custom application will need to handle the failure.

[Azure Container Instance]()You need to expose an endpoint for Blob Stroage Event Grid change, or use Azure SDK to listen on Azure IoT file upload message.

[App Service using Container]() run Linux and just job in the container as a loop. You need to expose an endpoint for Blob Stroage Event Grid change, or use Azure SDK to listen on Azure IoT file upload message.
[P1v3](https://azure.microsoft.com/en-us/pricing/details/app-service/linux/) 2cores	8GBmem 250GBdisk $137.97/month

[Linux](https://azure.microsoft.com/en-us/pricing/details/container-instances/) 2 cores, 8GB mem $85.13/month

[Function](https://azure.microsoft.com/en-us/pricing/details/functions/) premium plan, 2 cores, 8GB, 324.412/month

[Azure Function with Custom Container](https://docs.microsoft.com/en-us/azure/azure-functions/functions-create-function-linux-custom-image?tabs=bash%2Cportal&pivots=programming-language-python), least amount of coding, most expensive.
  * blob trigger- Azure blob trigger is not recommended, but if I use Event Grid trigger, I can't use function.json to bind the blob that triggered the function to blob input binding. I have to write extra code to access the blob. https://github.com/Azure/azure-functions-host/issues/7013
  * event grid blob trigger- Azure event grid blob trigger is still in preview, to install the beta extension, you need to install "func" can't just update host.json easily.

## How it works

Step 1: Set up Azure IoT Edge Blob Storage

For a `BlobTrigger` to work, you provide a path which dictates where the blobs are located inside your container, and can also help restrict the types of blobs you wish to return. For instance, you can set the path to `samples/{name}.png` to restrict the trigger to only the samples path and only blobs with ".png" at the end of their name.

## Known issue

* When testing locally in Docker, Azure blob trigger runs on all the blobs already in storage every time the function is updated (or when running local in a container, because host ID changes every time, and blob receipt is based on host id). https://github.com/Azure/azure-webjobs-sdk/issues/1327
* Storage binding doesn't support managed identity. https://github.com/Azure/azure-functions-host/issues/6423. And what about when function runs in a docker?
