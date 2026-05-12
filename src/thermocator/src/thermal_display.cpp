#include "thermocator/thermal_display.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>
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

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

ThermalDisplay::ThermalDisplay() {
    RCLCPP_INFO(_logger, "ThermalDisplay Docked onto RViz base");

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

    RCLCPP_DEBUG(_logger, "ThermalDisplay constructor done");
}

ThermalDisplay::~ThermalDisplay() {
    RCLCPP_INFO(_logger, "Removing ThermalDisplay — initialized=%s",
                initialized() ? "true" : "false");

    if (initialized()) {
        RCLCPP_DEBUG(_logger, "Destroying manual object");
        scene_manager_->destroyManualObject(_manual_object);

        RCLCPP_DEBUG(_logger, "Destroying child node");
        _child_node->getParentSceneNode()->removeAndDestroyChild(_child_node);

        if (_texture) {
            RCLCPP_DEBUG(_logger, "Removing texture");
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());
        }

        if (_material) {
            RCLCPP_DEBUG(_logger, "Removing material");
            Ogre::MaterialManager::getSingleton().remove(_material->getHandle());
        }

        RCLCPP_DEBUG(_logger, "Destroying resource group: %s",
                     _resource_group_name.c_str());
        Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(
            _resource_group_name);
    }

    RCLCPP_DEBUG(_logger, "ThermalDisplay destructor done");
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::onInitialize() {
    RCLCPP_INFO(_logger, "Initializing ...");

    MessageFilterDisplay::onInitialize();
    RCLCPP_DEBUG(_logger, "Base class initialized");

    if (!scene_manager_ || !scene_node_) {
        RCLCPP_FATAL(_logger, "scene_manager_ or scene_node_ is null — aborting");
        return;
    }

    static int instance_count = 0;
    _base_name = "ThermalDisplay_" + std::to_string(instance_count++);
    RCLCPP_INFO(_logger, "Instance name: %s", _base_name.c_str());

    // ── Resource group ─────────────────────────────────────────────────────────
    _resource_group_name = _base_name + "_group";
    RCLCPP_INFO(_logger, "Creating resource group: %s",
                _resource_group_name.c_str());
    Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
        _resource_group_name);

    // ── Scene node ─────────────────────────────────────────────────────────────
    RCLCPP_INFO(_logger, "Creating child scene node");
    _child_node = scene_node_->createChildSceneNode();

    // ── Manual object ──────────────────────────────────────────────────────────
    RCLCPP_INFO(_logger, "Creating manual object: %s_obj", _base_name.c_str());
    _manual_object = scene_manager_->createManualObject(_base_name + "_obj");
    _manual_object->setDynamic(true);
    _child_node->attachObject(_manual_object);
    RCLCPP_DEBUG(_logger, "Manual object attached");

    // ── Material ───────────────────────────────────────────────────────────────
    RCLCPP_INFO(_logger, "Creating material: %s_mat", _base_name.c_str());
    _material = Ogre::MaterialManager::getSingleton().create(
        _base_name + "_mat", _resource_group_name);
    _material->setReceiveShadows(false);

    auto *technique = _material->getTechnique(0);
    technique->setLightingEnabled(false);

    auto *pass = technique->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->createTextureUnitState();
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);
    RCLCPP_DEBUG(_logger, "Material configured");

    // ── QoS and topic ──────────────────────────────────────────────────────────
    qos_profile = rclcpp::QoS(1).transient_local().reliable();
    topic_property_->setValue("/thermal_map");

    RCLCPP_DEBUG(_logger, "onInitialize complete");
}

void ThermalDisplay::reset() {
    RCLCPP_DEBUG(_logger, "reset");
    MessageFilterDisplay::reset();
    _manual_object->clear();
    _last_width = 0;
    _last_height = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Property slots
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::onAlphaChanged() {
    RCLCPP_DEBUG(_logger, "Alpha changed to %.2f",
                 _alpha_property->getFloat());
}

void ThermalDisplay::onColorsChanged() {
    RCLCPP_DEBUG(_logger, "Colors changed");
}

// ─────────────────────────────────────────────────────────────────────────────
// Message processing
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::processMessage(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    RCLCPP_DEBUG(_logger,
                 "processMessage — frame=%s w=%u h=%u res=%.3f",
                 msg->header.frame_id.c_str(),
                 msg->info.width, msg->info.height,
                 msg->info.resolution);

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    Ogre::Vector3 sn_world = scene_node_->_getDerivedPosition();
    RCLCPP_DEBUG(_logger, "scene_node_ world pos: (%.3f, %.3f, %.3f)",
                 sn_world.x, sn_world.y, sn_world.z);

    if (!context_->getFrameManager()->transform(
            msg->header, msg->info.origin, position, orientation)) {
        RCLCPP_WARN(_logger,
                    "Could not transform frame [%s] to fixed frame — skipping",
                    msg->header.frame_id.c_str());
        return;
    }

    RCLCPP_DEBUG(_logger, "Transform OK: pos=(%.2f, %.2f, %.2f)",
                 position.x, position.y, position.z);

    const bool dims_changed =
        msg->info.width != _last_width ||
        msg->info.height != _last_height;

    if (dims_changed) {
        RCLCPP_DEBUG(_logger,
                     "Grid dimensions changed: %ux%u → %ux%u — rebuilding plane",
                     _last_width, _last_height,
                     msg->info.width, msg->info.height);
        updatePlane(*msg);
        _last_width = msg->info.width;
        _last_height = msg->info.height;
    }

    updateTexture(*msg);

    _child_node->setPosition(position);
    _child_node->setOrientation(orientation);

    RCLCPP_DEBUG(_logger, "processMessage done");
}

// ─────────────────────────────────────────────────────────────────────────────
// Texture update
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::updateTexture(const nav_msgs::msg::OccupancyGrid &grid) {
    const uint32_t w = grid.info.width;
    const uint32_t h = grid.info.height;

    RCLCPP_DEBUG(_logger, "updateTexture — w=%u h=%u", w, h);

    // Texture is stored transposed (h wide, w tall) to correct orientation
    if (!_texture ||
        _texture->getWidth() != h ||
        _texture->getHeight() != w) {
        RCLCPP_DEBUG(_logger,
                     "Creating texture %s_tex (%ux%u transposed)",
                     _base_name.c_str(), h, w);

        if (_texture) {
            RCLCPP_DEBUG(_logger, "Removing old texture");
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());
        }

        _texture = Ogre::TextureManager::getSingleton().createManual(
            _base_name + "_tex",
            _resource_group_name,
            Ogre::TEX_TYPE_2D,
            w, h,
            0,
            Ogre::PF_BYTE_RGBA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        _material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(_texture->getName());

        RCLCPP_DEBUG(_logger, "Texture bound to material");
    }

    const float alpha = _alpha_property->getFloat();
    const QColor cold = _cold_color_property->getColor();
    const QColor hot = _hot_color_property->getColor();
    const uint8_t a = static_cast<uint8_t>(alpha * 255.0f);

    const float dr = static_cast<float>(hot.red() - cold.red());
    const float dg = static_cast<float>(hot.green() - cold.green());
    const float db = static_cast<float>(hot.blue() - cold.blue());

    RCLCPP_DEBUG(_logger, "Locking pixel buffer");
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = _texture->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);

    uint8_t *data =
        static_cast<uint8_t *>(pixel_buffer->getCurrentLock().data);

    if (!data) {
        RCLCPP_ERROR(_logger, "Pixel buffer lock returned null data pointer");
        pixel_buffer->unlock();
        return;
    }

    int unknown_cells = 0;
    int cold_cells = 0;
    int hot_cells = 0;

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            const std::size_t grid_idx = static_cast<std::size_t>(row) * w + col;
            const std::size_t pixel_idx = grid_idx * 4;

            const int8_t value = grid.data[grid_idx];

            if (value == -1) {
                data[pixel_idx + 0] = 128;
                data[pixel_idx + 1] = 128;
                data[pixel_idx + 2] = 128;
                data[pixel_idx + 3] = 0;
                ++unknown_cells;
            } else {
                const float t = static_cast<float>(value) / 100.0f;
                data[pixel_idx + 0] = static_cast<uint8_t>(
                    static_cast<float>(cold.red()) + t * dr);
                data[pixel_idx + 1] = static_cast<uint8_t>(
                    static_cast<float>(cold.green()) + t * dg);
                data[pixel_idx + 2] = static_cast<uint8_t>(
                    static_cast<float>(cold.blue()) + t * db);
                data[pixel_idx + 3] = a;
                if (value == 0)
                    ++cold_cells;
                else
                    ++hot_cells;
            }
        }
    }

    pixel_buffer->unlock();

    RCLCPP_DEBUG(_logger,
                 "updateTexture done — unknown=%d cold=%d hot=%d",
                 unknown_cells, cold_cells, hot_cells);
}

// ─────────────────────────────────────────────────────────────────────────────
// Plane geometry
// ─────────────────────────────────────────────────────────────────────────────
void ThermalDisplay::updatePlane(const nav_msgs::msg::OccupancyGrid &grid) {
    const float w = static_cast<float>(grid.info.width) * grid.info.resolution;
    const float h = static_cast<float>(grid.info.height) * grid.info.resolution;

    // Local space only — scene node carries the world transform
    _manual_object->clear();
    _manual_object->begin(
        _material->getName(),
        Ogre::RenderOperation::OT_TRIANGLE_LIST,
        _resource_group_name);

    _manual_object->position(0.0f, 0.0f, 0.0f);
    _manual_object->textureCoord(0.0f, 0.0f);
    _manual_object->position(w, 0.0f, 0.0f);
    _manual_object->textureCoord(1.0f, 0.0f);
    _manual_object->position(w, h, 0.0f);
    _manual_object->textureCoord(1.0f, 1.0f);
    _manual_object->position(0.0f, h, 0.0f);
    _manual_object->textureCoord(0.0f, 1.0f);

    _manual_object->triangle(0, 1, 2);
    _manual_object->triangle(0, 2, 3);
    _manual_object->end();

    RCLCPP_DEBUG(_logger, "updatePlane done");
}

} // namespace thermocator

PLUGINLIB_EXPORT_CLASS(thermocator::ThermalDisplay, rviz_common::Display)
