#include "SimHUD.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"

#include "Vehicles/Multirotor/SimModeWorldMultiRotor.h"
#include "Vehicles/Car/SimModeCar.h"
#include "Vehicles/SkidSteer/SimModeSkidVehicle.h"
#include "Vehicles/ComputerVision/SimModeComputerVision.h"

#include "common/AirSimSettings.hpp"
#include <stdexcept>

ASimHUD::ASimHUD()
{
    /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
    static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_FlyChamsSimHUDWidget'"));
    /* ------------------------------------------------------------------------------------------------------ */
    widget_class_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
}

void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    try {
        UAirBlueprintLib::OnBeginPlay();
        initializeSettings();
        loadLevel();

        // Prevent a MavLink connection being established if changing levels
        if (map_changed_) return;

        setUnrealEngineSettings();
        createSimMode();
        createMainWidget();

        /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
        // Disable visibility of every subwindow
        for (int window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
            toggleSubwindowVisibility(window_index);
        }
        /* ------------------------------------------------------------------------------------------------------ */

        setupInputBindings();
        if (simmode_)
            simmode_->startApiServer();
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString("Error at startup: ", ex.what(), LogDebugLevel::Failure);
        //FGenericPlatformMisc::PlatformInit();
        //FGenericPlatformMisc::MessageBoxExt(EAppMsgType::Ok, TEXT("Error at Startup"), ANSI_TO_TCHAR(ex.what()));
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(), "Error");
    }
}

void ASimHUD::Tick(float DeltaSeconds)
{
    if (simmode_ && simmode_->EnableReport)
        widget_->updateDebugReport(simmode_->getDebugReport());
}

void ASimHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (simmode_)
        simmode_->stopApiServer();

    if (widget_) {
        widget_->Destruct();
        widget_ = nullptr;
    }
    if (simmode_) {
        simmode_->Destroy();
        simmode_ = nullptr;
    }

    UAirBlueprintLib::OnEndPlay();

    Super::EndPlay(EndPlayReason);
}

void ASimHUD::toggleRecordHandler()
{
    simmode_->toggleRecording();
}

void ASimHUD::inputEventToggleRecording()
{
    toggleRecordHandler();
}

void ASimHUD::inputEventToggleReport()
{
    simmode_->EnableReport = !simmode_->EnableReport;
    widget_->setReportVisible(simmode_->EnableReport);
}

void ASimHUD::inputEventToggleHelp()
{
    widget_->toggleHelpVisibility();
}

void ASimHUD::inputEventToggleTrace()
{
    simmode_->toggleTraceAll();
}

void ASimHUD::updateWidgetSubwindowVisibility()
{
    for (int window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
        APIPCamera* camera = subwindow_cameras_[window_index];
        ImageType camera_type = getSubWindowSettings().at(window_index).image_type;
        std::string annotation_name = getSubWindowSettings().at(window_index).annotation_name;

        if (camera_type == ImageType::Annotation) {
            if (simmode_->DoesAnnotationLayerExist(FString(annotation_name.c_str()))) {
                bool is_visible = getSubWindowSettings().at(window_index).visible && camera != nullptr;

                if (camera != nullptr) {
                    camera->setCameraTypeEnabled(camera_type, is_visible, annotation_name);
                    //sub-window captures don't count as a request, set bCaptureEveryFrame and bCaptureOnMovement to display so we can show correctly the subwindow
                    camera->setCameraTypeUpdate(camera_type, false, annotation_name);
                }

                widget_->setSubwindowVisibility(window_index,
                    is_visible,
                    is_visible ? camera->getRenderTarget(camera_type, false, annotation_name) : nullptr);
            }
        }
        else {
            bool is_visible = getSubWindowSettings().at(window_index).visible && camera != nullptr;

            if (camera != nullptr) {
                camera->setCameraTypeEnabled(camera_type, is_visible, annotation_name);
                //sub-window captures don't count as a request, set bCaptureEveryFrame and bCaptureOnMovement to display so we can show correctly the subwindow
                camera->setCameraTypeUpdate(camera_type, false, annotation_name);
            }

            widget_->setSubwindowVisibility(window_index,
                is_visible,
                is_visible ? camera->getRenderTarget(camera_type, false, annotation_name) : nullptr);
        }
    }
}

bool ASimHUD::isWidgetSubwindowVisible(int window_index)
{
    return widget_->getSubwindowVisibility(window_index) != 0;
}

void ASimHUD::toggleSubwindowVisibility(int window_index)
{
    getSubWindowSettings().at(window_index).visible = !getSubWindowSettings().at(window_index).visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleSubwindow0()
{
    toggleSubwindowVisibility(0);
}

void ASimHUD::inputEventToggleSubwindow1()
{
    toggleSubwindowVisibility(1);
}

void ASimHUD::inputEventToggleSubwindow2()
{
    toggleSubwindowVisibility(2);
}

void ASimHUD::inputEventToggleAll()
{
    /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
    getSubWindowSettings().at(0).visible = !getSubWindowSettings().at(0).visible;
    for (int window_index = 1; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
        getSubWindowSettings().at(window_index).visible = getSubWindowSettings().at(0).visible;
    }
    /* ------------------------------------------------------------------------------------------------------ */
    updateWidgetSubwindowVisibility();
}

void ASimHUD::createMainWidget()
{
    //create main widget
    if (widget_class_ != nullptr) {
        APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
        auto* pawn = player_controller->GetPawn();
        if (pawn) {
            std::string pawn_name = std::string(TCHAR_TO_ANSI(*pawn->GetName()));
            Utils::log(pawn_name);
        }
        else {
            UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("There were no compatible vehicles created for current SimMode! Check your settings.json."), "Error");
            UAirBlueprintLib::LogMessage(TEXT("There were no compatible vehicles created for current SimMode! Check your settings.json."), TEXT(""), LogDebugLevel::Failure);
        }

        widget_ = CreateWidget<USimHUDWidget>(player_controller, widget_class_);
    }
    else {
        widget_ = nullptr;
        UAirBlueprintLib::LogMessage(TEXT("Cannot instantiate BP_SimHUDWidget blueprint!"), TEXT(""), LogDebugLevel::Failure);
    }

    initializeSubWindows();

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->initializeForPlay();
    if (simmode_)
        widget_->setReportVisible(simmode_->EnableReport);
    widget_->setOnToggleRecordingHandler(std::bind(&ASimHUD::toggleRecordHandler, this));
    widget_->setRecordButtonVisibility(AirSimSettings::singleton().is_record_ui_visible);
    updateWidgetSubwindowVisibility();
}

void ASimHUD::setUnrealEngineSettings()
{
    //TODO: should we only do below on SceneCapture2D components and cameras?
    //avoid motion blur so capture images don't get
    //GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    //use two different methods to set console var because sometime it doesn't seem to work
    static const auto custom_depth_var = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);

    //Equivalent to enabling Custom Stencil in Project > Settings > Rendering > Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    //during startup we init stencil IDs to random hash and it takes long time for large environments
    //we get error that GameThread has timed out after 30 sec waiting on render thread
    static const auto render_timeout_var = IConsoleManager::Get().FindConsoleVariable(TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}

void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventToggleRecording", EKeys::R, this, &ASimHUD::inputEventToggleRecording);
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::Semicolon, this, &ASimHUD::inputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::inputEventToggleTrace);

    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow0", EKeys::One, this, &ASimHUD::inputEventToggleSubwindow0);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow1", EKeys::Two, this, &ASimHUD::inputEventToggleSubwindow1);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow2", EKeys::Three, this, &ASimHUD::inputEventToggleSubwindow2);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::inputEventToggleAll);
}

void ASimHUD::initializeSettings()
{
    std::string settingsText;
    if (getSettingsText(settingsText))
        AirSimSettings::initializeSettings(settingsText);
    else
        AirSimSettings::createDefaultSettingsFile();

    AirSimSettings::singleton().load(std::bind(&ASimHUD::getSimModeFromUser, this));
    for (const auto& warning : AirSimSettings::singleton().warning_messages) {
        UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
    }
    for (const auto& error : AirSimSettings::singleton().error_messages) {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, error, "settings.json");
    }
}

const std::vector<ASimHUD::AirSimSettings::SubwindowSetting>& ASimHUD::getSubWindowSettings() const
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::vector<ASimHUD::AirSimSettings::SubwindowSetting>& ASimHUD::getSubWindowSettings()
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::string ASimHUD::getSimModeFromUser()
{
    if (EAppReturnType::No == UAirBlueprintLib::ShowMessage(EAppMsgType::YesNo,
        "Would you like to use car/skid-vehicle simulation? Choose no to use quadrotor simulation.",
        "Choose Vehicle")) {
        return AirSimSettings::kSimModeTypeMultirotor;
    }
    else
        if (EAppReturnType::No == UAirBlueprintLib::ShowMessage(EAppMsgType::YesNo,
            "Would you like to use car simulation? Choose no to use skid-vehicle simulation.",
            "Choose Vehicle")) {
            return AirSimSettings::kSimModeTypeSkidVehicle;
        }
        else
            return AirSimSettings::kSimModeTypeCar;
}

void ASimHUD::loadLevel()
{
    UAirBlueprintLib::RunCommandOnGameThread([&]() { this->map_changed_ = UAirBlueprintLib::loadLevel(this->GetWorld(), FString(AirSimSettings::singleton().level_name.c_str())); }, true);
}

void ASimHUD::createSimMode()
{
    std::string simmode_name = AirSimSettings::singleton().simmode_name;

    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    //spawn at origin. We will use this to do global NED transforms, for ex, non-vehicle objects in environment
    if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
        simmode_ = this->GetWorld()->SpawnActor<ASimModeWorldMultiRotor>(FVector::ZeroVector,
            FRotator::ZeroRotator,
            simmode_spawn_params);
    else if (simmode_name == AirSimSettings::kSimModeTypeCar)
        simmode_ = this->GetWorld()->SpawnActor<ASimModeCar>(FVector::ZeroVector,
            FRotator::ZeroRotator,
            simmode_spawn_params);
    else if (simmode_name == AirSimSettings::kSimModeTypeSkidVehicle)
        simmode_ = this->GetWorld()->SpawnActor<ASimModeSkidVehicle>(FVector::ZeroVector,
            FRotator::ZeroRotator,
            simmode_spawn_params);
    else if (simmode_name == AirSimSettings::kSimModeTypeComputerVision)
        simmode_ = this->GetWorld()->SpawnActor<ASimModeComputerVision>(FVector::ZeroVector,
            FRotator::ZeroRotator,
            simmode_spawn_params);
    else {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("SimMode is not valid: ") + simmode_name, "Error");
        UAirBlueprintLib::LogMessageString("SimMode is not valid: ", simmode_name, LogDebugLevel::Failure);
    }
}

void ASimHUD::initializeSubWindows()
{
    if (!simmode_)
        return;

    auto default_vehicle_sim_api = simmode_->getVehicleSimApi();

    if (default_vehicle_sim_api) {
        auto camera_count = default_vehicle_sim_api->getCameraCount();

        /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
        //setup defaults
        for (int window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
            subwindow_cameras_[window_index] = nullptr;
        }
        /* ------------------------------------------------------------------------------------------------------ */
    }

    for (const auto& setting : getSubWindowSettings()) {
        APIPCamera* camera = simmode_->getCamera(msr::airlib::CameraDetails(setting.camera_name, setting.vehicle_name));
        if (camera)
            subwindow_cameras_[setting.window_index] = camera;
        else
            UAirBlueprintLib::LogMessageString("Invalid Camera settings in <SubWindows> element",
                std::to_string(setting.window_index),
                LogDebugLevel::Failure);
    }
}

FString ASimHUD::getLaunchPath(const std::string& filename)
{
    FString launch_rel_path = FPaths::LaunchDir();
    FString abs_path = FPaths::ConvertRelativePathToFull(launch_rel_path);
    return FPaths::Combine(abs_path, FString(filename.c_str()));
}

// Attempts to parse the settings text from one of multiple locations.
// First, check the command line for settings provided via "-s" or "--settings" arguments
// Next, check the executable's working directory for the settings file.
// Finally, check the user's documents folder.
// If the settings file cannot be read, throw an exception

bool ASimHUD::getSettingsText(std::string& settingsText)
{
    return (getSettingsTextFromCommandLine(settingsText) ||
        readSettingsTextFromFile(FString(msr::airlib::Settings::getExecutableFullPath("settings.json").c_str()), settingsText) ||
        readSettingsTextFromFile(getLaunchPath("settings.json"), settingsText) ||
        readSettingsTextFromFile(FString(msr::airlib::Settings::Settings::getUserDirectoryFullPath("settings.json").c_str()), settingsText));
}

// Attempts to parse the settings file path or the settings text from the command line
// Looks for the flag "-settings=". If it exists, settingsText will be set to the value.
// Example (Path): AirSim.exe -settings="C:\path\to\settings.json"
// Example (Text): AirSim.exe -settings={"foo":"bar"} -> settingsText will be set to {"foo":"bar"}
// Returns true if the argument is present, false otherwise.
bool ASimHUD::getSettingsTextFromCommandLine(std::string& settingsText)
{
    const TCHAR* commandLineArgs = FCommandLine::Get();
    FString settingsJsonFString;

    if (FParse::Value(commandLineArgs, TEXT("-settings="), settingsJsonFString, false)) {
        if (readSettingsTextFromFile(settingsJsonFString, settingsText)) {
            return true;
        }
        else {
            UAirBlueprintLib::LogMessageString("Loaded settings from commandline: ", TCHAR_TO_UTF8(*settingsJsonFString), LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsJsonFString);
            return true;
        }
    }

    return false;
}

bool ASimHUD::readSettingsTextFromFile(const FString& settingsFilepath, std::string& settingsText)
{
    bool found = FPaths::FileExists(settingsFilepath);
    if (found) {
        FString settingsTextFStr;
        bool readSuccessful = FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath);
        if (readSuccessful) {
            UAirBlueprintLib::LogMessageString("Loaded settings from ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
        }
        else {
            UAirBlueprintLib::LogMessageString("Cannot read file ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }

    return found;
}

/* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
//----------- Window APIs ----------/
void ASimHUD::setWindowImage(int window_index, const std::string& vehicle_name, const std::string& camera_name, const msr::airlib::Vector2r& crop_corner, const msr::airlib::Vector2r& crop_size)
{
    if (window_index >= AirSimSettings::kSubwindowCount)
    {
        UAirBlueprintLib::LogMessageString("Invalid window index ", std::to_string(window_index).c_str(), LogDebugLevel::Failure);
        return;
    }

    if (!simmode_)
        return;

    auto vehicle_sim_api = simmode_->getVehicleSimApi(vehicle_name);

    // Get and assign camera
    APIPCamera* camera = vehicle_sim_api->getCamera(camera_name);
    subwindow_cameras_[window_index] = camera;
    if (!camera) {
        updateSubWindow(window_index);
        return;
    }

    // Update camera type
    updateCameraType(camera);

    // Retrieve the image and crop dimensions
    int image_width = camera->getParams().capture_settings[0].width;
    int image_height = camera->getParams().capture_settings[0].height;
    int x = static_cast<int>(crop_corner.x());
    int y = static_cast<int>(crop_corner.y());
    int w = static_cast<int>(crop_size.x());
    int h = static_cast<int>(crop_size.y());

    // Validate that x, y, w, h are within the image bounds
    if (x < 0 || y < 0 || w < 0 || h < 0 || x + w > image_width || y + h > image_height)
    {
        UAirBlueprintLib::LogMessageString("Invalid cropping parameters: x=", std::to_string(x).c_str(),
            LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessageString(" y=", std::to_string(y).c_str(), LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessageString(" w=", std::to_string(w).c_str(), LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessageString(" h=", std::to_string(h).c_str(), LogDebugLevel::Failure);
        return;
    }

    // w == 0 or h == 0 is assumed as whole image
    if (w == 0 || h == 0)
    {
        updateSubWindow(window_index);
        return;
    }

    // If bounds are valid, update the sub-window with cropping
    updateSubWindowWithCropping(window_index, x, y, w, h);
}

void ASimHUD::initWindowDraw(int window_index, int width, int height)
{
    widget_->initializeSubwindowDraw(window_index, width, height);
}

void ASimHUD::beginWindowDraw(int window_index)
{
    widget_->beginSubwindowDraw(window_index);
}

void ASimHUD::endWindowDraw(int window_index)
{
    widget_->endSubwindowDraw(window_index);
}

void ASimHUD::drawWindowPoints(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float size)
{
    // Validate color_rgba size
    if (color_rgba.size() != 4) {
        UAirBlueprintLib::LogMessageString("Invalid drawing color size for window ", std::to_string(window_index).c_str(), LogDebugLevel::Failure);
        return;
    }

    // Convert color_rgba to FLinearColor
    FLinearColor color(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]);

    // Draw each point
    for (const auto& point : points) {
        widget_->drawSubwindowPoint(
            window_index,
            FVector2D(point.x(), point.y()),
            color,
            size);
    }
}

void ASimHUD::drawWindowLineStrip(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float thickness)
{
    // Validate color_rgba size
    if (color_rgba.size() != 4) {
        UAirBlueprintLib::LogMessageString("Invalid drawing color size for window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Convert color_rgba to FLinearColor
    FLinearColor color(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]);

    // Draw consecutive lines (0-1, 1-2, 2-3, ...)
    for (size_t i = 0; i < points.size() - 1; i++) {
        const auto& point_a = points[i];
        const auto& point_b = points[i + 1];

        // Use the widget's drawSubwindowLine function
        widget_->drawSubwindowLine(window_index,
            FVector2D(point_a.x(), point_a.y()),
            FVector2D(point_b.x(), point_b.y()),
            color,
            thickness);
    }

    // Draw the last line
    const auto& point_a = points[points.size() - 1];
    const auto& point_b = points[0];
    widget_->drawSubwindowLine(
        window_index,
        FVector2D(point_a.x(), point_a.y()),
        FVector2D(point_b.x(), point_b.y()),
        color,
        thickness);
}

void ASimHUD::drawWindowLineList(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float thickness)
{
    // Validate color_rgba size
    if (color_rgba.size() != 4) {
        UAirBlueprintLib::LogMessageString("Invalid drawing color size for window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Ensure there is an even number of points
    if (points.size() % 2 != 0) {
        UAirBlueprintLib::LogMessageString("Odd number of points provided for line list in window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Convert color_rgba to FLinearColor
    FLinearColor color(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]);

    // Draw lines between pairs of points (0-1, 2-3, 4-5, ...)
    for (size_t i = 0; i < points.size(); i += 2) {
        const auto& point_a = points[i];
        const auto& point_b = points[i + 1];

        // Use the widget's drawSubwindowLine function
        widget_->drawSubwindowLine(
            window_index,
            FVector2D(point_a.x(), point_a.y()),
            FVector2D(point_b.x(), point_b.y()),
            color,
            thickness);
    }
}

void ASimHUD::drawWindowBoxes(int window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color_rgba, float thickness)
{
    // Validate color_rgba size
    if (color_rgba.size() != 4) {
        UAirBlueprintLib::LogMessageString("Invalid drawing color size for window ", std::to_string(window_index).c_str(), LogDebugLevel::Failure);
        return;
    }

    // Validate input vectors
    if (corners.size() != sizes.size()) {
        UAirBlueprintLib::LogMessageString("Mismatch between corners and sizes vectors for window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Convert color_rgba to FLinearColor
    FLinearColor color(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]);

    // Draw each box
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto& corner = corners[i];
        const auto& size = sizes[i];
        widget_->drawSubwindowBox(
            window_index,
            FVector2D(corner.x(), corner.y()),
            FVector2D(size.x(), size.y()),
            color,
            thickness);
    }
}

void ASimHUD::drawWindowTags(int window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& text_color_rgba, const std::vector<float>& fill_color_rgba, const std::vector<float>& frame_color_rgba, float scale)
{
    // Validate color_rgba size
    if (text_color_rgba.size() != 4 || fill_color_rgba.size() != 4 || frame_color_rgba.size() != 4) {
        UAirBlueprintLib::LogMessageString("Invalid color size for window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Ensure strings and positions have the same size
    if (strings.size() != positions.size()) {
        UAirBlueprintLib::LogMessageString("Mismatch between strings and positions vectors for window " + std::to_string(window_index), "", LogDebugLevel::Failure);
        return;
    }

    // Convert colors to FLinearColor
    FLinearColor text_color(text_color_rgba[0], text_color_rgba[1], text_color_rgba[2], text_color_rgba[3]);
    FLinearColor fill_color(fill_color_rgba[0], fill_color_rgba[1], fill_color_rgba[2], fill_color_rgba[3]);
    FLinearColor frame_color(frame_color_rgba[0], frame_color_rgba[1], frame_color_rgba[2], frame_color_rgba[3]);

    // Draw each tag
    for (size_t i = 0; i < strings.size(); ++i) {
        const auto& string = strings[i];
        const auto& position = positions[i];

        // Convert std::string to FString
        FString fstring(TCHAR_TO_UTF8(*FString(string.c_str())));

        // Use the widget's drawSubwindowTag function
        widget_->drawSubwindowTag(
            window_index,
            fstring,
            FVector2D(position.x(), position.y()),
            text_color,
            fill_color,
            frame_color,
            scale);
    }
}

// Private methods
void ASimHUD::updateCameraType(APIPCamera* camera)
{
    if (camera) {
        camera->setCameraTypeEnabled(msr::airlib::ImageCaptureBase::ImageType::Scene, true, "");
        camera->setCameraTypeUpdate(msr::airlib::ImageCaptureBase::ImageType::Scene, false, "");
    }
}

void ASimHUD::updateSubWindow(int window_index)
{
    APIPCamera* camera = subwindow_cameras_[window_index];
    ImageType camera_type = getSubWindowSettings().at(window_index).image_type;
    std::string annotation_name = getSubWindowSettings().at(window_index).annotation_name;
    if (camera) {
        widget_->setSubwindowVisibility(window_index, true, camera->getRenderTarget(camera_type, false, annotation_name));
    }
    else {
        widget_->setSubwindowVisibility(window_index, false, nullptr);
        UAirBlueprintLib::LogMessageString("Invalid camera at window index ", std::to_string(window_index).c_str(), LogDebugLevel::Failure);
    }
}

void ASimHUD::updateSubWindowWithCropping(int window_index, int x, int y, int w, int h)
{
    APIPCamera* camera = subwindow_cameras_[window_index];
    ImageType camera_type = getSubWindowSettings().at(window_index).image_type;
    std::string annotation_name = getSubWindowSettings().at(window_index).annotation_name;
    if (camera) {
        widget_->setSubwindowVisibilityWithCropping(window_index, true, x, y, w, h, camera->getRenderTarget(camera_type, false, annotation_name));
    }
    else {
        widget_->setSubwindowVisibility(window_index, false, nullptr);
        UAirBlueprintLib::LogMessageString("Invalid camera at window index ", std::to_string(window_index).c_str(), LogDebugLevel::Failure);
    }
}
/* ------------------------------------------------------------------------------------------------------ */