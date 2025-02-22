#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include "api/ApiServerBase.hpp"
#include <memory>
#include "SimHUD.generated.h"

UENUM(BlueprintType)
enum class ESimulatorMode : uint8
{
    SIM_MODE_HIL UMETA(DisplayName = "Hardware-in-loop")
};

UCLASS()
class AIRSIM_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    void inputEventToggleRecording();
    void inputEventToggleReport();
    void inputEventToggleHelp();
    void inputEventToggleTrace();
    void inputEventToggleSubwindow0();
    void inputEventToggleSubwindow1();
    void inputEventToggleSubwindow2();
    void inputEventToggleAll();

    ASimHUD();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    /* ------------------------------------------- FLYINGCHAMELEONS ------------------------------------------ */
    //----------- Window APIs ----------/
    // Set image for a subwindow
    void setWindowImage(int window_index, const std::string& vehicle_name, const std::string& camera_name, const msr::airlib::Vector2r& crop_corner = msr::airlib::Vector2r(0.0, 0.0), const msr::airlib::Vector2r& crop_size = msr::airlib::Vector2r(0.0, 0.0));
    // Initialize draw with given size
    void initWindowDraw(int window_index, int width, int height);
    // Reset drawn objects
    void beginWindowDraw(int window_index);
    // Represent drawn objects
    void endWindowDraw(int window_index);
    // Plot points
    void drawWindowPoints(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float size);
    // Plot line for points 0-1, 1-2, 2-3
    void drawWindowLineStrip(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float thickness);
    // Plot line for points 0-1, 2-3, 4-5. Must be even number of points
    void drawWindowLineList(int window_index, const std::vector<msr::airlib::Vector2r>& points, const std::vector<float>& color_rgba, float thickness);
    // Plot boxes
    void drawWindowBoxes(int window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color_rgba, float thickness);
    // Plot tags
    void drawWindowTags(int window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& text_color_rgba, const std::vector<float>& fill_color_rgba, const std::vector<float>& frame_color_rgba, float scale);
    /* ------------------------------------------------------------------------------------------------------ */

protected:
    virtual void setupInputBindings();
    void toggleRecordHandler();
    void updateWidgetSubwindowVisibility();
    bool isWidgetSubwindowVisible(int window_index);
    void toggleSubwindowVisibility(int window_index);

private:
    void initializeSubWindows();
    void createSimMode();
    void initializeSettings();
    void setUnrealEngineSettings();
    void loadLevel();
    void createMainWidget();
    const std::vector<AirSimSettings::SubwindowSetting>& getSubWindowSettings() const;
    std::vector<AirSimSettings::SubwindowSetting>& getSubWindowSettings();

    bool getSettingsText(std::string& settingsText);
    bool getSettingsTextFromCommandLine(std::string& settingsText);
    bool readSettingsTextFromFile(const FString& fileName, std::string& settingsText);
    std::string getSimModeFromUser();

    static FString getLaunchPath(const std::string& filename);

    /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
    void updateCameraType(APIPCamera* camera);
    void updateSubWindow(int window_index);
    void updateSubWindowWithCropping(int window_index, int x, int y, int w, int h);
    /* ------------------------------------------------------------------------------------------------------ */

private:
    typedef common_utils::Utils Utils;
    UClass* widget_class_;

    UPROPERTY()
    USimHUDWidget* widget_;
    UPROPERTY()
    ASimModeBase* simmode_;

    APIPCamera* subwindow_cameras_[AirSimSettings::kSubwindowCount];
    bool map_changed_;
};
