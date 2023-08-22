//
// (c) Bit Parallel Ltd, August 2023
//

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <set>
#include <vector>

#include <opencv2/opencv.hpp>

int32_t aquireCalibrationPhotos(const int32_t cameraIndex, const std::filesystem::path& imagesPath)
{
    auto capture = cv::VideoCapture(cameraIndex, cv::CAP_V4L2);
    if (!capture.isOpened())
    {
        std::cout << "Failed to open the video stream for camera #" << cameraIndex << "\n";
        std::exit(0);
    }

    std::cout << "Hit RETURN to grab an image, ESC to quit...\n";
    cv::namedWindow("Calibration Image", 1);

    int32_t count = 0;
    auto frame = cv::Mat();
    while (capture.read(frame))
    {
        cv::imshow("Calibration Image", frame);

        const uint8_t key = cv::waitKey(5) & 0xff;
        if (key == 27) break;
        if (key == 13)
        {
            count++;

            const auto fileName = imagesPath.string() + "/calibration_image_" + std::to_string(count) + ".png";
            cv::imwrite(fileName, frame);
            std::cout << "Image grabbed #" << count << "\n";
        }
    }

    cv::destroyAllWindows();
    return count;
}

bool endsWith(const std::string& fullString, const std::string& ending)
{
	auto success = false;
	if (fullString.length() >= ending.length())
	{
		success = (fullString.compare(fullString.length() - ending.length(), ending.length(), ending) == 0);
	}

	return success;
}

void generateCalibration(const std::filesystem::path& imagesPath)
{
    const auto patternSize = cv::Size(13, 8);
    const auto windowSize = cv::Size(11, 11);
    const auto zeroZone = cv::Size(-1, -1);
    const auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    auto objectPoints = std::vector<std::vector<cv::Point3f>>();
    auto imagePoints = std::vector<std::vector<cv::Point2f>>();

    auto objP = std::vector<cv::Point3f>();
    for (int32_t i = 0; i < patternSize.height; i++)
        for (int32_t j = 0; j < patternSize.width; j++) objP.push_back(cv::Point3f(float(j), float(i), 0));

    // sort the filenames, this is just to print their names in order whilst generating the calibration
    //
    auto sortedFileSet = std::set<std::filesystem::path>();
    for (auto& entry : std::filesystem::directory_iterator(imagesPath)) sortedFileSet.insert(entry.path());

    cv::namedWindow("Calibration Image", 1);
    std::cout << "\nPress any key to progress to the next image\n";
    for (auto& path : sortedFileSet)
    {
        const std::string& file = path.filename().string();
        if (std::filesystem::is_regular_file(path) && endsWith(file, ".png"))
        {
            std::cout << "Processing image: " << file << "\n";

            // load an image and convert to b/w
            //
            auto image = cv::imread(path.string());
            cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

            // locate the corner patterns
            //
            auto corners = std::vector<cv::Point2f>();
            const auto patternFound = cv::findChessboardCorners(image, patternSize, corners);
            if (!patternFound)
            {
                std::cout << "No corners found in image: " << file << "\n";
                continue;
            }

            // improve on the accuracy
            //
            cv::cornerSubPix(image, corners, windowSize, zeroZone, criteria);
            imagePoints.push_back(corners);
            objectPoints.push_back(objP);

            cv::drawChessboardCorners(image, patternSize, corners, true);
            cv::imshow("Calibration Image", image);
            cv::waitKey(-1);
        }
    }

    cv::destroyAllWindows();

    auto cameraMatrix = cv::Mat();
    auto distortionCoeffs = cv::Mat();
    auto rotationVector = cv::Mat();
    auto translationVector = cv::Mat();
    const auto rmsReProjectionError = cv::calibrateCamera(objectPoints, imagePoints, cv::Size(1920, 1080), cameraMatrix, distortionCoeffs, rotationVector, translationVector);

    std::cout << "\n";
    std::cout << "RMS Projection Error: " << rmsReProjectionError << "\n\n";
    std::cout << "Camera Matrix:\n" << cameraMatrix << "\n\n";
    std::cout << "Distortion Coefficients:\n" << distortionCoeffs << "\n\n";
    std::cout << "Rotation Vector:\n" << rotationVector << "\n\n";
    std::cout << "Translation Vector:\n" << translationVector << "\n\n";

    // save the camera and distortion matrices
    //
    auto saveFile = cv::FileStorage(std::filesystem::current_path().string() + "/calibration.xml", cv::FileStorage::WRITE);
    saveFile << "Camera-Matrix" << cameraMatrix << "Distortion-Coefficients" << distortionCoeffs;
    saveFile.release();
}

void calibratedLiveView(cv::VideoCapture& capture)
{
    auto cameraMatrix = cv::Mat(), distortionCoeffs = cv::Mat();
    auto readFile = cv::FileStorage(std::filesystem::current_path().string() + "/calibration.xml", cv::FileStorage::READ);
    if (!readFile.isOpened())
    {
        std::cout << "Unable to open: calibration.xml\n";
        return;
    }

    readFile["Camera-Matrix"] >> cameraMatrix;
    readFile["Distortion-Coefficients"] >> distortionCoeffs;
    readFile.release();

    std::cout << "\n";
    std::cout << "Camera Matrix:\n" << cameraMatrix << "\n\n";
    std::cout << "Distortion Coefficients:\n" << distortionCoeffs << "\n\n";

    // the maps are cached for performance, to be used when re-mapping multiple images
    //
    const auto frameSize = cv::Size(capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    auto newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distortionCoeffs, frameSize, 0, frameSize);
    auto map1 = cv::Mat(), map2 = cv::Mat();
    cv::initUndistortRectifyMap(cameraMatrix, distortionCoeffs, cv::Mat(), newCameraMatrix, frameSize, CV_32FC1, map1, map2);

    cv::namedWindow("Calibrated Video", 1);
    auto remappedFrame = cv::Mat();
    auto frame = cv::Mat();
    while (capture.read(frame))
    {
        cv::remap(frame, remappedFrame, map1, map2, cv::INTER_LINEAR);
        cv::imshow("Calibrated Video", remappedFrame);

        const uint8_t key = cv::waitKey(5) & 0xff;
        if (key == 27) break;
    }

    cv::destroyAllWindows();
}

void displayHelp(const std::string& appName)
{
    std::cout << "Invalid arguments, please use:\n";
    std::cout << appName << " -c [#camera] [some-path] -d\n";
    std::cout << appName << " -t [#camera]\n";
}

int32_t main(int32_t argc, char** argv)
{
    const auto appName = std::string(argv[0]);
    if (argc == 1)
    {
        displayHelp(appName);
        return 0;
    }

    const auto command = std::string(argv[1]);
    if (command == "-c")
    {
        const auto imagesPath = std::filesystem::current_path() /= std::string(argv[3]);
        if (argc == 4)
        {
            if (!std::filesystem::is_empty(imagesPath))
            {
                std::cout << "The calibration images directory " << imagesPath << " is not empty\n";
                std::cout << "Perhaps add the -d option and retry\n";
                return 0;
            }
        }
        else if ((argc == 5))
        {
            if (std::string(argv[4]) == "-d")
            {
                for (auto& subPath : std::filesystem::directory_iterator(imagesPath)) std::filesystem::remove_all(subPath);
                std::cout << "The existing calibration images in " << imagesPath << " have been deleted\n";
            }
            else
            {
                displayHelp(appName);
                return 0;
            }
        }

        auto count = aquireCalibrationPhotos(std::stoi(std::string(argv[2])), imagesPath);
        std::cout << "Aquired " << count << " calibration images\n";

        // note, saves calibration.xml (will overwrite) in the current directory
        //
        try
        {
            generateCalibration(imagesPath);
            std::cout << "Successfully generated the camera calibration parameters\n";
        }
        catch (cv::Exception& ex)
        {
            std::cout << "Unable to generate the camera calibration parameters\n";
            std::cout << "Reason: " << ex.what() << "\n";
        }

        return 0;
    }

    if ((argc == 3) && (command == "-t"))
    {
        const auto cameraIndex = std::stoi(std::string(argv[2]));
        auto capture = cv::VideoCapture(cameraIndex, cv::CAP_V4L2);
        if (!capture.isOpened())
        {   
            std::cout << "Failed to open the video stream for camera #" << cameraIndex << "\n";
            return 0;
        }

        calibratedLiveView(capture);
        return 0;
    }

    displayHelp(appName);
    return 0;
}
