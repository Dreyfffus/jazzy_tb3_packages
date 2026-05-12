#pragma once

#include <cstdint>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTexture.h>

namespace thermocator {

class ThermalDisplay : public rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid> {
    Q_OBJECT

  public:
    ThermalDisplay();
    ~ThermalDisplay() override;

  protected:
    void onInitialize() override;
    void reset() override;

  private Q_SLOTS:
    void onAlphaChanged();
    void onColorsChanged();

  private:
    void processMessage(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override;

    void updateTexture(const nav_msgs::msg::OccupancyGrid &grid);

    void updatePlane(const nav_msgs::msg::OccupancyGrid &grid);

    Ogre::SceneNode *_child_node = nullptr;
    Ogre::ManualObject *_manual_object = nullptr;
    Ogre::TexturePtr _texture;
    Ogre::MaterialPtr _material;

    uint32_t _last_width = 0;
    uint32_t _last_height = 0;

    rviz_common::properties::FloatProperty *_alpha_property;
    rviz_common::properties::ColorProperty *_cold_color_property;
    rviz_common::properties::ColorProperty *_hot_color_property;

    std::string _resource_group_name;
    std::string _base_name;

    rclcpp::Logger _logger = rclcpp::get_logger("ThermalDisplay");
};

} // namespace thermocator
