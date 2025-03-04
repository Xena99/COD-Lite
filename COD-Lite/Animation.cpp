#include "Animation.h"

AnimationHandler::AnimationHandler() : currentAnimation(nullptr), currentFrame(0), animPlaying(true), animCount(0) {}

AnimationHandler::~AnimationHandler() {
    Unload();
}

bool AnimationHandler::FileExists(const char* filename) {
    FILE* file;
    if (fopen_s(&file, filename, "r") == 0)
    {
        fclose(file);
        return true;
    }
    return false;
}

void AnimationHandler::LoadAnimations(const char* animPath, Model& character) {
    if (!FileExists(animPath)) {
        TraceLog(LOG_ERROR, "Animation file not found: %s", animPath);
        return;
    }

    ModelAnimation* animations = LoadModelAnimations(animPath, &animCount);
    if (animCount == 0 || !animations) {
        TraceLog(LOG_ERROR, "Failed to load animations! Check file: %s", animPath);
        return;
    }

    for (int i = 0; i < animCount; i++) {
        std::string animName = animations[i].name;
        animationsMap[animName] = animations[i];
        TraceLog(LOG_INFO, "Loaded animation: %s", animName.c_str());
    }

    // Default to the first animation if available
    if (!animationsMap.empty()) {
        currentAnimation = &animationsMap["Idle"];
    }
}

void AnimationHandler::PlayAnimation(const std::string& name) {
    if (animationsMap.find(name) != animationsMap.end()) {
        currentAnimation = &animationsMap[name];
        currentFrame = 0;
        animPlaying = true;
        TraceLog(LOG_INFO, "Switched to animation: %s", name.c_str());
    }
    else {
        TraceLog(LOG_ERROR, "Animation not found: %s", name.c_str());
    }
}

void AnimationHandler::Update(Model& character) {
    if (!currentAnimation) {
        TraceLog(LOG_ERROR, "No animation selected!");
        return;
    }

    if (animPlaying) {
        currentFrame++;

        if (currentFrame < currentAnimation->frameCount) {
            UpdateModelAnimation(character, *currentAnimation, currentFrame);
        }
        else {
            currentFrame = 0;
        }
    }
}

void AnimationHandler::Unload() {
    animationsMap.clear();
}
