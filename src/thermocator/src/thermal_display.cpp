#include "thermocator/thermal_display.hpp"

#include <algorithm>
#include <string>

#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/render_system.hpp>

#include <pluginlib/class_list_macros.hpp>
namespace thermocator {
ThermalDisplay::ThermalDisplay() {
    // Properties are parented to `this` — the property panel in RViz2 owns them
    // SLOT macros fire when the user edits the value in the panel
    _alpha_property = new rviz_common::properties::FloatProperty(
        "Alpha", 0.7f,
        "Transparency of the thermal overlay",
        this, SLOT(onAlphaChanged()));
    _alpha_property->setMin(0.0f);
    _alpha_property->setMax(1.0f);

    _cold_color_property = new rviz_common::properties::ColorProperty(
        "Cold Color", QColor(0, 0, 255),
        "Color mapped to minimum temperature (value = 0)",
        this, SLOT(onColorsChanged()));

    _hot_color_property = new rviz_common::properties::ColorProperty(
        "Hot Color", QColor(255, 0, 0),
        "Color mapped to maximum temperature (value = 100)",
        this, SLOT(onColorsChanged()));
}

ThermalDisplay::~ThermalDisplay() {
    if (initialized()) {
        scene_manager_->destroyManualObject(_manual_object);
        _child_node->getParentSceneNode()->removeAndDestroyChild(_child_node);

        if (!_texture) {
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());
        }
        if (!_material) {
            Ogre::MaterialManager::getSingleton().remove(_material->getHandle());
        }

        Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(_resource_group_name);
    }
}

void ThermalDisplay::onInitialize() {
    MessageFilterDisplay::onInitialize();

    qos_profile = rclcpp::QoS(1).transient_local().reliable();

    topic_property_->setValue("/thermal_map");

    static int instance_count = 0;

    _resource_group_name = _base_name + "_group";
    Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
        _resource_group_name);

    _child_node = scene_node_->createChildSceneNode();

    _manual_object = scene_manager_->createManualObject(_base_name + "_obj");
    _manual_object->setDynamic(true);
    _child_node->attachObject(_manual_object);

    _material = Ogre::MaterialManager::getSingleton().create(
        _base_name + "_mat",
        _resource_group_name); // ← not DEFAULT_RESOURCE_GROUP_NAME

    _material->setReceiveShadows(false);
    auto *technique = _material->getTechnique(0);
    technique->setLightingEnabled(false);
    auto *pass = technique->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->createTextureUnitState();
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);
}

void ThermalDisplay::reset() {
    MessageFilterDisplay::reset();
    _manual_object->clear();
    _last_width = 0;
    _last_height = 0;
}

void ThermalDisplay::onAlphaChanged() {
    // Alpha change only affects pixel colors — no geometry rebuild needed
    // Will take effect on next message. If you want immediate update,
    // you would need to cache the last message and re-call updateTexture()
}

void ThermalDisplay::onColorsChanged() {
    // Same as above — next message picks up the new color scheme
}

void ThermalDisplay::processMessage(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    if (!context_->getFrameManager()->transform(
            msg->header,
            msg->info.origin,
            position,
            orientation)) {
        RCLCPP_WARN(rclcpp::get_logger("ThermalDisplay"),
                    "Could not transform from [%s] to fixed frame — skipping message",
                    msg->header.frame_id.c_str());
        return;
    }

    const bool dims_changed =
        msg->info.width != _last_width ||
        msg->info.height != _last_height;

    if (dims_changed) {
        updatePlane(*msg);
        _last_width = msg->info.width;
        _last_height = msg->info.height;
    }

    updateTexture(*msg);

    _child_node->setPosition(position);
    _child_node->setOrientation(orientation);
}

void ThermalDisplay::updateTexture(const nav_msgs::msg::OccupancyGrid &grid) {
    const uint32_t w = grid.info.width;
    const uint32_t h = grid.info.height;

    if (_texture ||
        _texture->getWidth() != w ||
        _texture->getHeight() != h) {
        if (!_texture) {
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());
        }

        _texture = Ogre::TextureManager::getSingleton().createManual(
            _base_name + "_tex",
            _resource_group_name, // ← use isolated group
            Ogre::TEX_TYPE_2D,
            w, h, 0,
            Ogre::PF_BYTE_RGBA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        _material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(_texture->getName());
    }

    const float alpha = _alpha_property->getFloat();
    const QColor cold = _cold_color_property->getColor();
    const QColor hot = _hot_color_property->getColor();
    const uint8_t a = static_cast<uint8_t>(alpha * 255.0f);

    const float dr = static_cast<float>(hot.red() - cold.red());
    const float dg = static_cast<float>(hot.green() - cold.green());
    const float db = static_cast<float>(hot.blue() - cold.blue());

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = _texture->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);

    uint8_t *data =
        static_cast<uint8_t *>(pixel_buffer->getCurrentLock().data);

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            // OccupancyGrid: row 0 = bottom of world
            // Ogre texture:  row 0 = top of image
            // → flip row index vertically
            const std::size_t grid_idx =
                static_cast<std::size_t>(row) * w + col;
            const std::size_t tex_row = h - 1 - row;
            const std::size_t pixel_idx = (tex_row * w + col) * 4;

            const int8_t value = grid.data[grid_idx];

            if (value == -1) {
                // Unknown cell — fully transparent
                data[pixel_idx + 0] = 128;
                data[pixel_idx + 1] = 128;
                data[pixel_idx + 2] = 128;
                data[pixel_idx + 3] = 0;
            } else {
                // Normalise [0, 100] → [0.0, 1.0] and lerp between cold and hot
                const float t = static_cast<float>(value) / 100.0f;

                data[pixel_idx + 0] =
                    static_cast<uint8_t>(static_cast<float>(cold.red()) + t * dr);
                data[pixel_idx + 1] =
                    static_cast<uint8_t>(static_cast<float>(cold.green()) + t * dg);
                data[pixel_idx + 2] =
                    static_cast<uint8_t>(static_cast<float>(cold.blue()) + t * db);
                data[pixel_idx + 3] = a;
            }
        }
    }

    pixel_buffer->unlock();
}

void ThermalDisplay::updatePlane(const nav_msgs::msg::OccupancyGrid &grid) {
    const float w = static_cast<float>(grid.info.width) * grid.info.resolution;
    const float h = static_cast<float>(grid.info.height) * grid.info.resolution;

    _manual_object->clear();
    _manual_object->begin(
        _material->getName(),
        Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Single quad covering the full grid extent
    // Origin (0,0) is the bottom-left corner — matches OccupancyGrid origin
    // UV coordinates are flipped to account for the texture vertical flip above
    //
    //  3 ──────── 2
    //  │  (grid)  │
    //  0 ──────── 1
    //
    _manual_object->position(0.0f, 0.0f, 0.0f);
    _manual_object->textureCoord(0.0f, 1.0f);
    _manual_object->position(w, 0.0f, 0.0f);
    _manual_object->textureCoord(1.0f, 1.0f);
    _manual_object->position(w, h, 0.0f);
    _manual_object->textureCoord(1.0f, 0.0f);
    _manual_object->position(0.0f, h, 0.0f);
    _manual_object->textureCoord(0.0f, 0.0f);

    _manual_object->triangle(0, 1, 2);
    _manual_object->triangle(0, 2, 3);

    _manual_object->end();
};
} // namespace thermocator
PLUGINLIB_EXPORT_CLASS(thermocator::ThermalDisplay, rviz_common::Display)
