#pragma once

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/String.h>
#include <Corrade/PluginManager/AbstractPlugin.h>

namespace planmt {

class AbstractEncoder: public Corrade::PluginManager::AbstractPlugin {
    public:
        static Corrade::Containers::StringView pluginInterface() {
            using namespace Corrade::Containers::Literals;
            return "encoders"_s;
        }

        static Corrade::Containers::Array<Corrade::Containers::String> pluginSearchPaths() {
            return {Corrade::InPlaceInit, {""}};
        }

        explicit AbstractEncoder(Corrade::PluginManager::AbstractManager& manager, Corrade::Containers::StringView plugin):
            AbstractPlugin{manager, plugin} {}

        virtual Corrade::Containers::String name() const = 0;
        virtual int channelCount() const = 0;
        virtual bool supportsStereo() const = 0;
};

}