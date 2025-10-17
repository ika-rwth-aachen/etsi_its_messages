#pragma once

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayManager.h>

#include <QImage>
#include <QColor>

/**
 * Move the copied classes to out namespace.
 */
namespace rviz_plugin {
class OverlayObject;

/**
 * This class is copied from project jsk_rviz_plugins to avoid installation process.
 */
class ScopedPixelBuffer {
 public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject &overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor &bg_color);
  virtual QImage getQImage(OverlayObject &overlay, QColor &bg_color);
 protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
 private:

};

/**
 * This class is copied from project jsk_rviz_plugins to avoid installation process.
 */
class OverlayObject {
 public:
  explicit OverlayObject(const std::string &name);
  virtual ~OverlayObject();

  virtual std::string getName();
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady();
  virtual bool updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(double left, double top);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible();
  virtual unsigned int getTextureWidth();
  virtual unsigned int getTextureHeight();
 protected:
  const std::string name_;
  Ogre::Overlay *overlay_;
  Ogre::PanelOverlayElement *panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;

 private:

};

}  // namespace rviz_plugin
