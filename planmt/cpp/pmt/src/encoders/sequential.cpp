
#include <Corrade/PluginManager/AbstractManager.h>

#include "encoder_base_plugin.hpp"

namespace planmt { 

class Sequential: public AbstractEncoder {
    public:
        explicit Sequential(Corrade::PluginManager::AbstractManager& manager, Corrade::Containers::StringView plugin):
            AbstractEncoder{manager, plugin} {}

        Corrade::Containers::String name() const override { return "Sequential"; }
        int channelCount() const override { return 2; }
        bool supportsStereo() const override { return true; }
};

}

CORRADE_PLUGIN_REGISTER(Sequential, planmt::Sequential, "src.encoders.sequential/1.0")