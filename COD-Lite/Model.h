#pragma once

#include "PCH.h"

class GameModel {
public:
    Model model;
    Texture2D texture;
    Vector3 position;

    GameModel(const char* modelPath, const char* texturePath);
    ~GameModel();

    void Draw();
    void Unload();
};

