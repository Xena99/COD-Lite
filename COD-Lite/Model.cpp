#include "Model.h"

GameModel::GameModel(const char* modelPath, const char* texturePath)
{
    model = LoadModel(modelPath);

    texture = LoadTexture(texturePath);
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    
}

GameModel::~GameModel()
{
    Unload();
}

void GameModel::Draw()
{
    DrawModel(model, position, 0.1f, WHITE);
}

void GameModel::Unload()
{
    UnloadModel(model);
    UnloadTexture(texture);
}
