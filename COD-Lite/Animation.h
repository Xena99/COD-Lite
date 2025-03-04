#pragma once
#include "PCH.h"

class AnimationHandler {
public:
    AnimationHandler();
    ~AnimationHandler();

    void LoadAnimations(const char* animPath, Model& character);
    void PlayAnimation(const std::string& name);
    void Update(Model& character);
    void Unload();

private:
    std::unordered_map<std::string, ModelAnimation> animationsMap;
    ModelAnimation* currentAnimation;
    int currentFrame;
    bool animPlaying;
    int animCount;

    bool FileExists(const char* filename);
};
