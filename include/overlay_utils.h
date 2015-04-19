#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgrePanelOverlayElement.h>
#include <OGRE/OgreOverlayElement.h>
#include <OGRE/OgreOverlayContainer.h>

#include <QImage>
#include <QColor>

namespace flobotics_finger_rviz_force_plot
{
  class OverlayObject;

  class ScopedPixelBuffer
  {
  public:
    ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
    virtual ~ScopedPixelBuffer();
    virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
    virtual QImage getQImage(unsigned int width, unsigned int height);
    virtual QImage getQImage(OverlayObject& overlay);
    virtual QImage getQImage(unsigned int width, unsigned int height, QColor& bg_color);
    virtual QImage getQImage(OverlayObject& overlay, QColor& bg_color);
  protected:
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
  private:

  };


  // this is a class for put overlay object on rviz 3D panel.
  // This class suppose to be instantiated in onInitialize method
  // of rviz::Display class.
  class OverlayObject
  {
  public:
    typedef boost::shared_ptr<OverlayObject> Ptr;

    OverlayObject(const std::string& name);
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
    Ogre::Overlay* overlay_;
    Ogre::PanelOverlayElement* panel_;
    Ogre::MaterialPtr panel_material_;
    Ogre::TexturePtr texture_;

  private:

  };
}
  // Ogre::Overlay* createOverlay(std::string name);
  // Ogre::PanelOverlayElement* createOverlayPanel(Ogre::Overlay* overlay);
  // Ogre::MaterialPtr createOverlayMaterial(Ogre::Overlay* overlay);
