#pragma once

#include "../params.h"

struct ICP_LM_Params : public Params
{
    ICP_LM_Params(std::filesystem::path path) 
    : Params(path), distance_threshold(), scale(), voxel_size(), iterations() 
    {
        parse(); 
    }

    ICP_LM_Params(std::string path)
    : Params(path), distance_threshold(), scale(), voxel_size(), iterations()
    { 
        parse(); 
    }

    virtual ~ICP_LM_Params() = default;

    void parse() final
    {
        const auto& config_data = toml::find(data_, "icp_lm");
        distance_threshold = toml::find<float>(config_data, "distance_threshold");
        scale = toml::find<float>(config_data, "scale");
        voxel_size = toml::find<float>(config_data, "voxel_size");
        iterations = toml::find<size_t>(config_data, "iterations");

    }

    float distance_threshold;
    float scale;
    float voxel_size;
    size_t iterations;
};