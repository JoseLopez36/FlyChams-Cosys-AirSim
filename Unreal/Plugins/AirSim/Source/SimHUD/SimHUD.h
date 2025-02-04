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

    /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
    // FlyChams APIs
    void setAgentSubWindows(const std::string& vehicle_name);
    void setSubWindowImage(int window_index, const std::string& vehicle_name, const std::string& camera_name);
    void setSubWindowImageWithCropping(int window_index, int x, int y, int w, int h, const std::string& vehicle_name, const std::string& camera_name);
    void drawTargetsInMap(const std::vector<int>& x, const std::vector<int>& y);
    void drawClustersInMap(const std::vector<int>& x, const std::vector<int>& y, const std::vector<int>& r);
    void drawAgentsInMap(const std::vector<int>& x, const std::vector<int>& y);
    void drawTargetsInSubWindow(int window_index, const std::vector<int>& x, const std::vector<int>& y, const std::vector<int>& w, const std::vector<int>& h);
    void drawClustersInSubWindow(int window_index, const std::vector<int>& x, const std::vector<int>& y, const std::vector<int>& r);
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
