#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "PIPCamera.h"
#include <functional>
#include "SimHUDWidget.generated.h"

UCLASS()
class AIRSIM_API USimHUDWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Event handler")
    void onToggleRecordingButtonClick();

public:
    typedef std::function<void(void)> OnToggleRecording;

    //TODO: Tick is not working
    //virtual void Tick_Implementation(FGeometry MyGeometry, float InDeltaTime) override;

    void updateDebugReport(const std::string& text);
    void setReportVisible(bool is_visible);
    void toggleHelpVisibility();

    void setOnToggleRecordingHandler(OnToggleRecording handler);

public:
    //below are implemented in Blueprint. The return value is forced to be
    //bool even when not needed because of Unreal quirk that if return value
    //is not there then below are treated as events instead of overridable functions
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setSubwindowVisibility(int window_index, bool is_visible, UTextureRenderTarget2D* render_target);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    int getSubwindowVisibility(int window_index);

    /* -------------------------------------------FLYINGCHAMELEONS ------------------------------------------ */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setSubwindowVisibilityWithCropping(int window_index, bool is_visible, int x, int y, int w, int h, UTextureRenderTarget2D* render_target);
    // Draw control methods
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool initializeSubwindowDraw(int window_index, int width, int height);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool beginSubwindowDraw(int window_index);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool endSubwindowDraw(int window_index);
    // Draw methods
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool drawSubwindowPoint(int window_index, const FVector2D& point, const FLinearColor& color, float size);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool drawSubwindowLine(int window_index, const FVector2D& point_a, const FVector2D& point_b, const FLinearColor& color, float thickness);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool drawSubwindowBox(int window_index, const FVector2D& corner, const FVector2D& size, const FLinearColor& color, float thickness);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool drawSubwindowTag(int window_index, const FString& string, const FVector2D& position, const FLinearColor& text_color, const FLinearColor& fill_color, const FLinearColor& frame_color, float scale);
    /* ------------------------------------------------------------------------------------------------------ */

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setRecordButtonVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getRecordButtonVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool initializeForPlay();

protected:
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getReportContainerVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setHelpContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getHelpContainerVisibility();

    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportText(const FString& text);

private:
    OnToggleRecording on_toggle_recording_;
};
