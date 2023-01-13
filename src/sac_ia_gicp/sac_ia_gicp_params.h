#pragma once

#include "params.h"

struct SAC_IA_GICP_Params : public Params
{
    SAC_IA_GICP_Params(std::filesystem::path path) 
    : Params(path), gicp(), fpfh(), sac_ia() 
    {
        parse(); 
    }

    SAC_IA_GICP_Params(std::string path)
    : Params(path), gicp(), fpfh(), sac_ia() 
    { 
        parse(); 
    }

    virtual ~SAC_IA_GICP_Params() = default;

    struct gicp
    {
        float resolution;
    } gicp;

    struct fpfh
    {
        int normal_k;
    } fpfh;

    struct sac_ia
    {
        float resolution;
    } sac_ia;

    void parse() final
    {
        const auto& gicp_data = toml::find(data_, "gicp");
        gicp.resolution = toml::find<float>(gicp_data, "resolution");

        const auto& fpfh_data = toml::find(data_, "fpfh");
        fpfh.normal_k = toml::find<int>(fpfh_data, "normal_k");

        const auto& sac_ia_data = toml::find(data_, "sac_ia");
        sac_ia.resolution = toml::find<float>(sac_ia_data, "resolution");
    }
};
