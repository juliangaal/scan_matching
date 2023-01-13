#pragma once

#include <toml.hpp>
#include <filesystem>

struct Params
{
    Params(std::filesystem::path path) 
    : data_(toml::parse(path)) 
    {

    }

    Params(std::string path) 
    : data_(toml::parse(path)) 
    {

    }

    virtual ~Params() = default;
    virtual void parse() = 0;
    toml::basic_value<TOML11_DEFAULT_COMMENT_STRATEGY, std::unordered_map, std::vector> data_;
};