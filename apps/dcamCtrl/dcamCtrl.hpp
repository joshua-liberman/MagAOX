/** \file dcamCtrl.hpp
  * \brief The MagAO-X Hamamatsu Q-CMOS camera controller.
  *
  * \author Joshua Liberman (jliberman@arizona.edu)
  *
  * \ingroup dcamCtrl_files
  */

#ifndef dcamCtrl_hpp
#define dcamCtrl_hpp


//#include <ImageStruct.h>
#include <ImageStreamIO/ImageStreamIO.h>

#include <dcam_advanced.h>

#include "../../libMagAOX/libMagAOX.hpp" //Note this is included on command line to trigger pch
#include "../../magaox_git_version.h"

#define DEBUG

#ifdef DEBUG
#define BREADCRUMB  std::cerr << __FILE__ << " " << __LINE__ << "\n";
#else
#define BREADCRUMB
#endif

inline
std::string DcamEnum2String( DcamEnumeratedType type, piint value )
{
    const pichar* string;
    Dcam_GetEnumerationString( type, value, &string );
    std::string str(string);
    Dcam_DestroyString( string );

    return str;
}

namespace MagAOX
{
namespace app
{

int readoutParams( piint & adcQual,
                   piflt & adcSpeed,
                   const std::string & rosn
                 )
{
   if(rosn == "cmos_00_1MHz")
   {
      adcQual = DcamAdcQuality_LowNoise;
      adcSpeed = 0.1;
   }
   else if(rosn == "cmos_01MHz")
   {
      adcQual = DcamAdcQuality_LowNoise;
      adcSpeed = 1;
   }
   else if(rosn == "cmos_05MHz")
   {
      adcQual = DcamAdcQuality_ElectronMultiplied;
      adcSpeed = 5;
   }
   else if(rosn == "cmos_10MHz")
   {
      adcQual = DcamAdcQuality_ElectronMultiplied;
      adcSpeed = 10;
   }
   else if(rosn == "cmos_20MHz")
   {
      adcQual = DcamAdcQuality_ElectronMultiplied;
      adcSpeed = 20;
   }
   else if(rosn == "cmos_30MHz")
   {
      adcQual = DcamAdcQuality_ElectronMultiplied;
      adcSpeed = 30;
   }
   else
   {
      return -1;
   }
   
   return 0;
}

int vshiftParams( piflt & vss,
                  const std::string & vsn
                )
{
   if(vsn == "0_7us")
   {
      vss = 0.7;
   }
   else if(vsn == "1_2us")
   {
      vss = 1.2;
   }
   else if(vsn == "2_0us")
   {
      vss = 2.0;
   }
   else if(vsn == "5_0us")
   {
      vss = 5.0;
   }
   else
   {
      return -1;
   }
   
   return 0;
}


/** \defgroup dcamCtrl Hamamatsu Q-CMOS Camera
  * \brief Control of a Hamamatsu Q-CMOS Camera.
  *
  * <a href="../handbook/operating/software/apps/dcamCtrl.html">Application Documentation</a>
  *
  * \ingroup apps
  *
  */

/** \defgroup dcamCtrl_files Hamamatsu Q-CMOS Camera Files
  * \ingroup dcamCtrl
  */

/** MagAO-X application to control a Hamamatsu Q-CMOS
  *
  * \ingroup dcamCtrl
  *
  * \todo Config item for ImageStreamIO name filename
  * \todo implement ImageStreamIO circular buffer, with config setting
  */
class dcamCtrl : public MagAOXApp<>, public dev::stdCamera<dcamCtrl>, public dev::frameGrabber<dcamCtrl>, public dev::dssShutter<dcamCtrl>, public dev::telemeter<dcamCtrl>
{

   friend class dev::stdCamera<dcamCtrl>;
   friend class dev::frameGrabber<dcamCtrl>;
   friend class dev::dssShutter<dcamCtrl>;
   friend class dev::telemeter<dcamCtrl>;

   typedef MagAOXApp<> MagAOXAppT;

public:
   /** \name app::dev Configurations
     *@{
     */
   static constexpr bool c_stdCamera_tempControl = true; ///< app::dev config to tell stdCamera to expose temperature controls
   
   static constexpr bool c_stdCamera_temp = true; ///< app::dev config to tell stdCamera to expose temperature
   
   static constexpr bool c_stdCamera_readoutSpeed = true; ///< app::dev config to tell stdCamera to expose readout speed controls
   
   static constexpr bool c_stdCamera_vShiftSpeed = true; ///< app:dev config to tell stdCamera to expose vertical shift speed control

   static constexpr bool c_stdCamera_emGain = true; ///< app::dev config to tell stdCamera to expose EM gain controls 

   static constexpr bool c_stdCamera_exptimeCtrl = true; ///< app::dev config to tell stdCamera to expose exposure time controls
   
   static constexpr bool c_stdCamera_fpsCtrl = false; ///< app::dev config to tell stdCamera not to expose FPS controls

   static constexpr bool c_stdCamera_fps = true; ///< app::dev config to tell stdCamera not to expose FPS status
   
   static constexpr bool c_stdCamera_synchro = false; ///< app::dev config to tell stdCamera to not expose synchro mode controls
   
   static constexpr bool c_stdCamera_usesModes = false; ///< app:dev config to tell stdCamera not to expose mode controls
   
   static constexpr bool c_stdCamera_usesROI = true; ///< app:dev config to tell stdCamera to expose ROI controls

   static constexpr bool c_stdCamera_cropMode = false; ///< app:dev config to tell stdCamera to expose Crop Mode controls
   
   static constexpr bool c_stdCamera_hasShutter = true; ///< app:dev config to tell stdCamera to expose shutter controls
      
   static constexpr bool c_stdCamera_usesStateString = false; ///< app::dev confg to tell stdCamera to expose the state string property
   
   static constexpr bool c_frameGrabber_flippable = true; ///< app:dev config to tell framegrabber this camera can be flipped
   
   ///@}
   
protected:

   /** \name configurable parameters
     *@{
     */
   std::string m_serialNumber; ///< The camera's identifying serial number


   ///@}

   int m_depth {0};
   
   piint m_timeStampMask {DcamTimeStampsMask_ExposureStarted}; // time stamp at end of exposure
   pi64s m_tsRes; // time stamp resolution
   piint m_frameSize;
   double m_camera_timestamp {0.0};
   piflt m_FrameRateCalculation;
   piflt m_ReadOutTimeCalculation;
   
   
   
   

   

   DcamHandle m_cameraHandle {0};
   DcamHandle m_modelHandle {0};

   DcamAcquisitionBuffer m_acqBuff;
   DcamAvailableData m_available;

   std::string m_cameraName;
   std::string m_cameraModel;

public:

   ///Default c'tor
   dcamCtrl();

   ///Destructor
   ~dcamCtrl() noexcept;

   /// Setup the configuration system (called by MagAOXApp::setup())
   virtual void setupConfig();

   /// load the configuration system results (called by MagAOXApp::setup())
   virtual void loadConfig();

   /// Startup functions
   /** Sets up the INDI vars.
     *
     */
   virtual int appStartup();

   /// Implementation of the FSM for the Siglent SDG
   virtual int appLogic();

   /// Implementation of the on-power-off FSM logic
   virtual int onPowerOff();

   /// Implementation of the while-powered-off FSM
   virtual int whilePowerOff();

   /// Do any needed shutdown tasks.  Currently nothing in this app.
   virtual int appShutdown();

protected:
   int getDcamParameter( piint & value,
                          DcamParameter parameter
                        );

   int getDcamParameter( piflt & value,
                          DcamParameter parameter
                        );

   int setDcamParameter( DcamParameter parameter,
                          pi64s value,
                          bool commit = true
                        );

   int setDcamParameter( DcamParameter parameter,
                          piint value,
                          bool commit = true
                        );

   int setDcamParameter( DcamHandle handle,
                          DcamParameter parameter,
                          piflt value,
                          bool commit = true
                        );

   int setDcamParameter( DcamHandle handle,
                          DcamParameter parameter,
                          piint value,
                          bool commit = true
                        );


   int setDcamParameter( DcamParameter parameter,
                          piflt value,
                          bool commit = true
                        );

   int setDcamParameterOnline( DcamHandle handle,
                                DcamParameter parameter,
                                piflt value
                              );

   int setDcamParameterOnline( DcamParameter parameter,
                                piflt value
                              );

   int setDcamParameterOnline( DcamHandle handle,
                                DcamParameter parameter,
                                piint value
                              );

   int setDcamParameterOnline( DcamParameter parameter,
                                piint value
                              );
   
   int connect();

   int getAcquisitionState();

   int getTemps();

   // stdCamera interface:
   
   //This must set the power-on default values of
   /* -- m_ccdTempSetpt
    * -- m_currentROI 
    */
   int powerOnDefaults();
   
   int setTempControl();
   int setTempSetPt();
   int setReadoutSpeed();
   int setVShiftSpeed();
   int setEMGain();
   int setExpTime();
   int capExpTime(piflt& exptime);
   int setFPS();

   /// Check the next ROI
   /** Checks if the target values are valid and adjusts them to the closest valid values if needed.
     *
     * \returns 0 if successful
     * \returns -1 otherwise
     */
   int checkNextROI();

   int setNextROI();

   /// Sets the shutter state, via call to dssShutter::setShutterState(int) [stdCamera interface]
   /**
     * \returns 0 always
     */
   int setShutter(int sh);
   
   //Framegrabber interface:
   int configureAcquisition();
   float fps();
   int startAcquisition();
   int acquireAndCheckValid();
   int loadImageIntoStream(void * dest);
   int reconfig();


   //INDI:
protected:

   pcf::IndiProperty m_indiP_readouttime;

public:
   INDI_NEWCALLBACK_DECL(dcamCtrl, m_indiP_adcquality);

   /** \name Telemeter Interface
     * 
     * @{
     */ 
   int checkRecordTimes();
   
   int recordTelem( const telem_stdcam * );
   
   
   ///@}
};

inline
dcamCtrl::dcamCtrl() : MagAOXApp(MAGAOX_CURRENT_SHA1, MAGAOX_REPO_MODIFIED)
{
   m_powerMgtEnabled = true;

   m_acqBuff.memory_size = 0;
   m_acqBuff.memory = 0;

   m_defaultReadoutSpeed  = "cmos_05MHz";
   m_readoutSpeedNames = {"cmos_00_1MHz", "cmos_01MHz", "cmos_05MHz", "cmos_10MHz", "cmos_20MHz", "cmos_30MHz"};
   m_readoutSpeedNameLabels = {"CCD 0.1 MHz", "CCD 1 MHz", "QCMOS 5 MHz", "QCMOS 10 MHz", "QCMOS 20 MHz", "QCMOS 30 MHz"};
   
   m_defaultVShiftSpeed = "1_2us";
   m_vShiftSpeedNames = {"0_7us", "1_2us", "2_0us", "5_0us"};
   m_vShiftSpeedNameLabels = {"0.7 us", "1.2 us", "2.0 us", "5.0 us"};
   
   
   m_default_x = 511.5; 
   m_default_y = 511.5; 
   m_default_w = 1024;  
   m_default_h = 1024;  
      
   m_full_x = 511.5; 
   m_full_y = 511.5; 
   m_full_w = 1024; 
   m_full_h = 1024; 
   
   m_maxEMGain = 1000;
   
   return;
}

inline
dcamCtrl::~dcamCtrl() noexcept
{
   if(m_acqBuff.memory)
   {
      free(m_acqBuff.memory);
   }

   return;
}

inline
void dcamCtrl::setupConfig()
{
   config.add("camera.serialNumber", "", "camera.serialNumber", argType::Required, "camera", "serialNumber", false, "int", "The identifying serial number of the camera.");

   dev::stdCamera<dcamCtrl>::setupConfig(config);
   dev::frameGrabber<dcamCtrl>::setupConfig(config);
   dev::dssShutter<dcamCtrl>::setupConfig(config);
   dev::telemeter<dcamCtrl>::setupConfig(config);
}

inline
void dcamCtrl::loadConfig()
{

   config(m_serialNumber, "camera.serialNumber");

   dev::stdCamera<dcamCtrl>::loadConfig(config);
   dev::frameGrabber<dcamCtrl>::loadConfig(config);
   dev::dssShutter<dcamCtrl>::loadConfig(config);
   dev::telemeter<dcamCtrl>::loadConfig(config);
   

}

inline
int dcamCtrl::appStartup()
{

   // DELETE ME
   //m_outfile = fopen("/home/xsup/test2.txt", "w");

   createROIndiNumber( m_indiP_readouttime, "readout_time", "Readout Time (s)");
   indi::addNumberElement<float>( m_indiP_readouttime, "value", 0.0, std::numeric_limits<float>::max(), 0.0,  "%0.1f", "readout time");
   registerIndiPropertyReadOnly( m_indiP_readouttime );

   
   m_minTemp = -55;
   m_maxTemp = 25;
   m_stepTemp = 0;
   
   m_minROIx = 0;
   m_maxROIx = 1023;
   m_stepROIx = 0;
   
   m_minROIy = 0;
   m_maxROIy = 1023;
   m_stepROIy = 0;
   
   m_minROIWidth = 1;
   m_maxROIWidth = 1024;
   m_stepROIWidth = 4;
   
   m_minROIHeight = 1;
   m_maxROIHeight = 1024;
   m_stepROIHeight = 1;
   
   m_minROIBinning_x = 1;
   m_maxROIBinning_x = 32;
   m_stepROIBinning_x = 1;
   
   m_minROIBinning_y = 1;
   m_maxROIBinning_y = 1024;
   m_stepROIBinning_y = 1;
   
   if(dev::stdCamera<dcamCtrl>::appStartup() < 0)
   {
      return log<software_critical,-1>({__FILE__,__LINE__});
   }
   
   if(dev::frameGrabber<dcamCtrl>::appStartup() < 0)
   {
      return log<software_critical,-1>({__FILE__,__LINE__});
   }

   if(dev::dssShutter<dcamCtrl>::appStartup() < 0)
   {
      return log<software_critical,-1>({__FILE__,__LINE__});
   }

   if(dev::telemeter<dcamCtrl>::appStartup() < 0)
   {
      return log<software_error,-1>({__FILE__,__LINE__});
   }
   
   return 0;

}

inline
int dcamCtrl::appLogic()
{
   //and run stdCamera's appLogic
   if(dev::stdCamera<dcamCtrl>::appLogic() < 0)
   {
      return log<software_error, -1>({__FILE__, __LINE__});
   }
   
   //first run frameGrabber's appLogic to see if the f.g. thread has exited.
   if(dev::frameGrabber<dcamCtrl>::appLogic() < 0)
   {
      return log<software_error, -1>({__FILE__, __LINE__});
   }

   //and run dssShutter's appLogic
   if(dev::dssShutter<dcamCtrl>::appLogic() < 0)
   {
      return log<software_error, -1>({__FILE__, __LINE__});
   }


   if( state() == stateCodes::NOTCONNECTED || state() == stateCodes::NODEVICE || state() == stateCodes::ERROR)
   {
      m_reconfig = true; //Trigger a f.g. thread reconfig.

      //Might have gotten here because of a power off.
      if(powerState() != 1 || powerStateTarget() != 1) return 0;

      std::unique_lock<std::mutex> lock(m_indiMutex);
      if(connect() < 0)
      {
         if(powerState() != 1 || powerStateTarget() != 1) return 0;
         log<software_error>({__FILE__, __LINE__});
      }

      if(state() != stateCodes::CONNECTED) return 0;
   }

   if( state() == stateCodes::CONNECTED )
   {
      //Get a lock
      std::unique_lock<std::mutex> lock(m_indiMutex);

      if( getAcquisitionState() < 0 )
      {
         if(powerState() != 1 || powerStateTarget() != 1) return 0;
         return log<software_error,0>({__FILE__,__LINE__});
      }

      if( setTempSetPt() < 0 ) //m_ccdTempSetpt already set on power on
      {
         if(powerState() != 1 || powerStateTarget() != 1) return 0;
         return log<software_error,0>({__FILE__,__LINE__});
      }

      
      if(frameGrabber<dcamCtrl>::updateINDI() < 0)
      {
         return log<software_error,0>({__FILE__,__LINE__});
      }
      
      setDcamParameter(m_modelHandle, DcamParameter_DisableCoolingFan, DcamCoolingFanStatus_On);
      
      
   }

   if( state() == stateCodes::READY || state() == stateCodes::OPERATING )
   {
      //Get a lock if we can
      std::unique_lock<std::mutex> lock(m_indiMutex, std::try_to_lock);

      //but don't wait for it, just go back around.
      if(!lock.owns_lock()) return 0;

      if(getAcquisitionState() < 0)
      {
         if(powerState() != 1 || powerStateTarget() != 1) return 0;

         state(stateCodes::ERROR);
         return 0;
      }

      

      if(getTemps() < 0)
      {
         if(powerState() != 1 || powerStateTarget() != 1) return 0;

         state(stateCodes::ERROR);
         return 0;
      }

      if(stdCamera<dcamCtrl>::updateINDI() < 0)
      {
         return log<software_error,0>({__FILE__,__LINE__});
      }
      
      if(frameGrabber<dcamCtrl>::updateINDI() < 0)
      {
         return log<software_error,0>({__FILE__,__LINE__});
      }

      if(telemeter<dcamCtrl>::appLogic() < 0)
      {
         log<software_error>({__FILE__, __LINE__});
         return 0;
      }

   }

   //Fall through check?
   return 0;

}

inline
int dcamCtrl::onPowerOff()
{
   std::lock_guard<std::mutex> lock(m_indiMutex);

   if(m_cameraHandle)
   {
      Dcam_CloseCamera(m_cameraHandle);
      m_cameraHandle = 0;
   }

   Dcam_UninitializeLibrary();

   if(dssShutter<dcamCtrl>::onPowerOff() < 0)
   {
      log<software_error>({__FILE__, __LINE__});
   }

   if(stdCamera<dcamCtrl>::onPowerOff() < 0)
   {
      log<software_error>({__FILE__, __LINE__});
   }
   
   return 0;
}

inline
int dcamCtrl::whilePowerOff()
{
   if(dssShutter<dcamCtrl>::whilePowerOff() < 0)
   {
      log<software_error>({__FILE__, __LINE__});
   }

   if(stdCamera<dcamCtrl>::onPowerOff() < 0 )
   {
      log<software_error>({__FILE__, __LINE__});
   }
   
   return 0;
}

inline
int dcamCtrl::appShutdown()
{
   dev::frameGrabber<dcamCtrl>::appShutdown();

   if(m_cameraHandle)
   {
      Dcam_CloseCamera(m_cameraHandle);
      m_cameraHandle = 0;
   }

   Dcam_UninitializeLibrary();

   ///\todo error check these base class fxns.
   dev::frameGrabber<dcamCtrl>::appShutdown();
   dev::dssShutter<dcamCtrl>::appShutdown();

   return 0;
}

inline
int dcamCtrl::getDcamParameter( piint & value,
                                  DcamParameter parameter
                                )
{
   DcamError error = Dcam_GetParameterIntegerValue( m_cameraHandle, parameter, &value );

   if(MagAOXAppT::m_powerState == 0) return -1; //Flag error but don't log

   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   return 0;
}

inline
int dcamCtrl::getDcamParameter( piflt & value,
                                  DcamParameter parameter
                                )
{
   DcamError error = Dcam_GetParameterFloatingPointValue( m_cameraHandle, parameter, &value );

   if(MagAOXAppT::m_powerState == 0) return -1; //Flag error but don't log

   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   return 0;
}

inline
int dcamCtrl::setDcamParameter( DcamParameter parameter,
                                  pi64s value,
                                  bool commit
                                )
{
   DcamError error = Dcam_SetParameterLargeIntegerValue( m_cameraHandle, parameter, value );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   if(!commit) return 0;
   
   const DcamParameter* failed_parameters;
   piint failed_parameters_count;

   error = Dcam_CommitParameters( m_cameraHandle, &failed_parameters, &failed_parameters_count );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }
   
   for( int i=0; i< failed_parameters_count; ++i)
   {
      if( failed_parameters[i] ==  parameter)
      {
         Dcam_DestroyParameters( failed_parameters );
         return log<text_log,-1>( "Parameter not committed");
      }
   }
   
   Dcam_DestroyParameters( failed_parameters );

   return 0;
}

inline
int dcamCtrl::setDcamParameter( DcamHandle handle,
                                  DcamParameter parameter,
                                  piflt value,
                                  bool commit
                                )
{
   DcamError error = Dcam_SetParameterFloatingPointValue( handle, parameter, value );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   if(!commit) return 0;
   
   const DcamParameter* failed_parameters;
   piint failed_parameters_count;

   error = Dcam_CommitParameters( handle, &failed_parameters, &failed_parameters_count );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   for( int i=0; i< failed_parameters_count; ++i)
   {
      if( failed_parameters[i] ==  parameter)
      {
         Dcam_DestroyParameters( failed_parameters );
         return log<text_log,-1>( "Parameter not committed");
      }
   }

   Dcam_DestroyParameters( failed_parameters );

   return 0;
}

inline
int dcamCtrl::setDcamParameter( DcamHandle handle,
                                  DcamParameter parameter,
                                  piint value,
                                  bool commit
                                )
{
   DcamError error = Dcam_SetParameterIntegerValue( handle, parameter, value );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   if(!commit) return 0;
   
   const DcamParameter* failed_parameters;
   piint failed_parameters_count;

   error = Dcam_CommitParameters( handle, &failed_parameters, &failed_parameters_count );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   for( int i=0; i< failed_parameters_count; ++i)
   {
      if( failed_parameters[i] ==  parameter)
      {
         Dcam_DestroyParameters( failed_parameters );
         return log<text_log,-1>( "Parameter not committed");
      }
   }

   Dcam_DestroyParameters( failed_parameters );

   return 0;
}

inline
int dcamCtrl::setDcamParameter( DcamParameter parameter,
                                  piflt value,
                                  bool commit
                                )
{
   return setDcamParameter( m_cameraHandle, parameter, value, commit);
}

inline
int dcamCtrl::setDcamParameter( DcamParameter parameter,
                                  piint value,
                                  bool commit
                                )
{
   return setDcamParameter( m_cameraHandle, parameter, value, commit);
}

inline
int dcamCtrl::setDcamParameterOnline( DcamHandle handle,
                                        DcamParameter parameter,
                                        piflt value
                                       )
{
   DcamError error = Dcam_SetParameterFloatingPointValueOnline( handle, parameter, value );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   return 0;
}

inline
int dcamCtrl::setDcamParameterOnline( DcamParameter parameter,
                                        piflt value
                                       )
{
   return setDcamParameterOnline(m_cameraHandle, parameter, value);
}

inline
int dcamCtrl::setDcamParameterOnline( DcamHandle handle,
                                        DcamParameter parameter,
                                        piint value
                                       )
{
   DcamError error = Dcam_SetParameterIntegerValueOnline( handle, parameter, value );
   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      return -1;
   }

   return 0;
}

inline
int dcamCtrl::setDcamParameterOnline( DcamParameter parameter,
                                        piint value
                                       )
{
   return setDcamParameterOnline(m_cameraHandle, parameter, value);
}

inline
int dcamCtrl::connect()
{

   DcamError error;
   DcamCameraID * id_array;
   piint id_count;

   if(m_acqBuff.memory)
   {
      free(m_acqBuff.memory);
      m_acqBuff.memory = NULL;
      m_acqBuff.memory_size = 0;
   }

   Dcam_UninitializeLibrary();

   //Have to initialize the library every time.  Otherwise we won't catch a newly booted camera.
   Dcam_InitializeLibrary();

   if(m_cameraHandle)
   {
      Dcam_CloseCamera(m_cameraHandle);
      m_cameraHandle = 0;
   }

   Dcam_GetAvailableCameraIDs(const_cast<const DcamCameraID **>(&id_array), &id_count);

   if(powerState() != 1 || powerStateTarget() != 1) return 0;

   if(id_count == 0)
   {
      Dcam_DestroyCameraIDs(id_array);

      Dcam_UninitializeLibrary();

      state(stateCodes::NODEVICE);
      if(!stateLogged())
      {
         log<text_log>("no P.I. Cameras available.");
      }
      return 0;
   }

   for(int i=0; i< id_count; ++i)
   {
      if( std::string(id_array[i].serial_number) == m_serialNumber )
      {
         log<text_log>("Camera was found.  Now connecting.");

         error = DcamAdvanced_OpenCameraDevice(&id_array[i], &m_cameraHandle);
         if(error == DcamError_None)
         {
            m_cameraName = id_array[i].sensor_name;
            m_cameraModel = DcamEnum2String(DcamEnumeratedType_Model, id_array[i].model);

            error = DcamAdvanced_GetCameraModel( m_cameraHandle, &m_modelHandle );
            if( error != DcamError_None )
            {
               log<software_error>({__FILE__, __LINE__, "failed to get camera model"});
            }

            state(stateCodes::CONNECTED);
            log<text_log>("Connected to " + m_cameraName + " [S/N " + m_serialNumber + "]");

            Dcam_DestroyCameraIDs(id_array);

            m_readoutSpeedNameSet = m_defaultReadoutSpeed;
            m_vShiftSpeedNameSet = m_defaultVShiftSpeed;
            
            return 0;
         }
         else
         {
            if(powerState() != 1 || powerStateTarget() != 1) return 0;

            state(stateCodes::ERROR);
            if(!stateLogged())
            {
               log<software_error>({__FILE__,__LINE__, 0, error, "Error connecting to camera."});
            }

            Dcam_DestroyCameraIDs(id_array);
            
            Dcam_UninitializeLibrary();
            return -1;
         }
      }
   }

   state(stateCodes::NODEVICE);
   if(!stateLogged())
   {
      log<text_log>("Camera not found in available ids.");
   }

   Dcam_DestroyCameraIDs(id_array);


   Dcam_UninitializeLibrary();


   return 0;
}


inline
int dcamCtrl::getAcquisitionState()
{
   pibln running = false;

   DcamError error = Dcam_IsAcquisitionRunning(m_cameraHandle, &running);

   if(MagAOXAppT::m_powerState == 0) return 0;

   if(error != DcamError_None)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      state(stateCodes::ERROR);
      return -1;
   }

   if(running) state(stateCodes::OPERATING);
   else state(stateCodes::READY);

   if(!running)
   {
      log<text_log>("acqusition stopped. restarting", logPrio::LOG_ERROR);
      m_reconfig = true;
   }

   return 0;

}

inline
int dcamCtrl::getTemps()
{
   piflt currTemperature;

   if(getDcamParameter(currTemperature, DcamParameter_SensorTemperatureReading) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;

      log<software_error>({__FILE__, __LINE__});
      state(stateCodes::ERROR);
      return -1;
   }

   m_ccdTemp = currTemperature;
   
   //DcamSensorTemperatureStatus
   piint status;

   if(getDcamParameter( status, DcamParameter_SensorTemperatureStatus ) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;

      log<software_error>({__FILE__, __LINE__});
      state(stateCodes::ERROR);
      return -1;
   }

   if(status == 1) 
   {
      m_tempControlStatus = true;
      m_tempControlOnTarget = false;
      m_tempControlStatusStr = "UNLOCKED";
   }
   else if(status == 2) 
   {
      m_tempControlStatus = true;
      m_tempControlOnTarget = true;
      m_tempControlStatusStr = "LOCKED";
   }
   else if(status == 3) 
   {
      m_tempControlStatus = false;
      m_tempControlOnTarget = false;
      m_tempControlStatusStr = "FAULTED";
      log<text_log>("temperature control faulted", logPrio::LOG_ALERT);
   }
   else
   {
      m_tempControlStatus = false;
      m_tempControlOnTarget = false;
      m_tempControlStatusStr = "UNKNOWN";
   }   

   recordCamera();

   return 0;

}

inline
int dcamCtrl::setFPS()
{
   return 0;
}

inline 
int dcamCtrl::powerOnDefaults()
{
   m_ccdTempSetpt = -55; //This is the power on setpoint

   m_currentROI.x = 511.5;
   m_currentROI.y = 511.5;
   m_currentROI.w = 1024;
   m_currentROI.h = 1024;
   m_currentROI.bin_x = 1;
   m_currentROI.bin_y = 1;

   m_readoutSpeedName = "cmos_05MHz";
   m_vShiftSpeedName = "1_2us";
   return 0;
}

inline 
int dcamCtrl::setTempControl()
{
   //Always on
   m_tempControlStatus = true;
   m_tempControlStatusSet = true;
   updateSwitchIfChanged(m_indiP_tempcont, "toggle", pcf::IndiElement::On, INDI_IDLE);
   recordCamera(true);
   return 0;
}

inline 
int dcamCtrl::setTempSetPt()
{
   ///\todo bounds check here.
   m_reconfig = true;

   recordCamera(true);
   return 0;
}

inline 
int dcamCtrl::setReadoutSpeed()
{
   m_reconfig = true;
   recordCamera(true);
   return 0;
}

inline 
int dcamCtrl::setVShiftSpeed()
{
   m_reconfig = true;
   recordCamera(true);
   return 0;
}

inline
int dcamCtrl::setEMGain()
{
   piint adcQual;
   piflt adcSpeed;
   
   if(readoutParams(adcQual, adcSpeed, m_readoutSpeedName) < 0)
   {
      log<software_error>({__FILE__, __LINE__, "Invalid readout speed: " + m_readoutSpeedNameSet});
      state(stateCodes::ERROR);
      return -1;
   }
   
   if(adcQual != DcamAdcQuality_ElectronMultiplied)
   {
      m_emGain = 1;
      m_adcSpeed = adcSpeed;
      recordCamera(true);
      log<text_log>("Attempt to set EM gain while in conventional amplifier.", logPrio::LOG_NOTICE);
      return 0;
   }
   
   piint emg = m_emGainSet;
   if(emg < 0)
   {
      emg = 0;
      log<text_log>("EM gain limited to 0", logPrio::LOG_WARNING);
   }
   
   if(emg > m_maxEMGain)
   {
      emg = m_maxEMGain;
      log<text_log>("EM gain limited to maxEMGain = " + std::to_string(emg), logPrio::LOG_WARNING);
   }
   
   recordCamera(true);
   if(setDcamParameterOnline(m_modelHandle, DcamParameter_AdcEMGain, emg) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting EM gain"});
      return -1;
   }
   
   piint AdcEMGain;
   if(getDcamParameter(AdcEMGain, DcamParameter_AdcEMGain) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      return log<software_error,-1>({__FILE__, __LINE__, "could not get AdcEMGain"});
   }
   m_emGain = AdcEMGain;
   m_adcSpeed = adcSpeed;
   recordCamera(true);
   return 0;
}

inline
int dcamCtrl::setExpTime()
{
   long intexptime = m_expTimeSet * 1000 * 10000 + 0.5;
   piflt exptime = ((double)intexptime)/10000;
   capExpTime(exptime);

   int rv;
   
   recordCamera(true);

   if(state() == stateCodes::OPERATING)
   {
      rv = setDcamParameterOnline(m_modelHandle, DcamParameter_ExposureTime, exptime);      
   }
   else
   {
      rv = setDcamParameter(m_modelHandle, DcamParameter_ExposureTime, exptime);
   }

   if(rv < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting exposure time"});
      return -1;
   }

   m_expTime = exptime/1000.0;

   recordCamera(true);

   updateIfChanged(m_indiP_exptime, "current", m_expTime, INDI_IDLE);

   if(getDcamParameter(m_FrameRateCalculation, DcamParameter_FrameRateCalculation) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "could not get FrameRateCalculation"});
   }
   m_fps = m_FrameRateCalculation;
   
   recordCamera(true);

   return 0;
}

inline
int dcamCtrl::capExpTime(piflt& exptime)
{
   // cap at minimum possible value
   if(exptime < m_ReadOutTimeCalculation)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<text_log>("Got exposure time " + std::to_string(exptime) + " ms but min value is " + std::to_string(m_ReadOutTimeCalculation) + " ms");
      long intexptime = m_ReadOutTimeCalculation * 10000 + 0.5;
      exptime = ((double)intexptime)/10000;
   }
   
   return 0;
}

inline
int dcamCtrl::checkNextROI()
{
   return 0;
}

//Set ROI property to busy if accepted, set toggle to Off and Idlw either way.
//Set ROI actual 
//Update current values (including struct and indiP) and set to OK when done
inline 
int dcamCtrl::setNextROI()
{   
   m_reconfig = true;

   updateSwitchIfChanged(m_indiP_roi_set, "request", pcf::IndiElement::Off, INDI_IDLE);
   
   return 0;
   
}

inline 
int dcamCtrl::setShutter( int sh )
{
   return dssShutter<dcamCtrl>::setShutterState(sh);
}

inline
int dcamCtrl::configureAcquisition()
{

   piint readoutStride;
   piint framesPerReadout;
   piint frameStride;
   //piint frameSize;
   piint pixelBitDepth;

   m_camera_timestamp = 0; // reset tracked timestamp

   std::unique_lock<std::mutex> lock(m_indiMutex);


   // Time stamp handling
   if(Dcam_SetParameterIntegerValue(m_modelHandle, DcamParameter_TimeStamps,  m_timeStampMask) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__,__LINE__, "Could not set time stamp mask"});
   }
   if(Dcam_GetParameterLargeIntegerValue(m_modelHandle, DcamParameter_TimeStampResolution, &m_tsRes) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__,__LINE__, "Could not get timestamp resolution"}) ;
   }

   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // Check Frame Transfer
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   piint cmode;
   if(getDcamParameter(cmode, DcamParameter_ReadoutControlMode) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__,__LINE__, "could not get Readout Control Mode"});
      return -1;
   }

   if( cmode != DcamReadoutControlMode_FrameTransfer)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__,__LINE__, "Readout Control Mode not configured for frame transfer"}) ;
      return -1;
   }

   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // Temperature
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   if(setDcamParameter( DcamParameter_SensorTemperatureSetPoint, m_ccdTempSetpt) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting temperature setpoint"});
      state(stateCodes::ERROR);
      return -1;
   }

   //log<text_log>( "Set temperature set point: " + std::to_string(m_ccdTempSetpt) + " C");

   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // ADC Speed and Quality
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   piint adcQual;
   piflt adcSpeed;
   
   if(readoutParams(adcQual, adcSpeed, m_readoutSpeedNameSet) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Invalid readout speed: " + m_readoutSpeedNameSet});
      state(stateCodes::ERROR);
      return -1;
   }
   
   if( setDcamParameter(m_modelHandle, DcamParameter_AdcSpeed, adcSpeed, false) < 0) //don't commit b/c it will error if quality mismatched
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting ADC Speed"});
      //state(stateCodes::ERROR);
      //return -1;
   }
   
   if( setDcamParameter(m_modelHandle, DcamParameter_AdcQuality, adcQual) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting ADC Quality"});
      state(stateCodes::ERROR);
      return -1;
   }
   m_adcSpeed = adcSpeed;
   m_readoutSpeedName = m_readoutSpeedNameSet;
   log<text_log>( "Readout speed set to: " + m_readoutSpeedNameSet);

   if(adcQual == DcamAdcQuality_LowNoise)
   {
      m_emGain = 1.0;
      m_emGainSet = 1.0;
   }

   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // Vertical Shift Rate
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   
   piflt vss;
   if(vshiftParams(vss, m_vShiftSpeedNameSet) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Invalid vertical shift speed: " + m_vShiftSpeedNameSet});
      state(stateCodes::ERROR);
      return -1;
   }
   
   if( setDcamParameter(m_modelHandle, DcamParameter_VerticalShiftRate, vss) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error setting Vertical Shift Rate"});
      state(stateCodes::ERROR);
      return -1;
   }

   m_vShiftSpeedName = m_vShiftSpeedNameSet;
   m_vshiftSpeed = vss;
   log<text_log>( "Vertical Shift Rate set to: " + m_vShiftSpeedName);

   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // Dimensions
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   
   DcamRois  nextrois;
   DcamRoi nextroi;
   
   nextrois.roi_array = &nextroi;
   nextrois.roi_count = 1;
   
   int roi_err = false;
   if(m_currentFlip == fgFlipLR || m_currentFlip == fgFlipUDLR)
   {
      nextroi.x = ((1023-m_nextROI.x) - 0.5*( (float) m_nextROI.w - 1.0));
   }
   else
   {
      nextroi.x = (m_nextROI.x - 0.5*( (float) m_nextROI.w - 1.0));
   }
   
   if(nextroi.x < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to x center < 0"});
      roi_err = true;
   }

   if(nextroi.x > 1023) 
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to x center > 1023"});
      roi_err = true;
   }
   

   if(m_currentFlip == fgFlipUD || m_currentFlip == fgFlipUDLR)
   {
      nextroi.y = ((1023 - m_nextROI.y) - 0.5*( (float) m_nextROI.h - 1.0));
   }
   else
   {
      nextroi.y = (m_nextROI.y - 0.5*( (float) m_nextROI.h - 1.0));
   }
   
   if(nextroi.y < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to y center < 0"});
      roi_err = true;
   }

   if(nextroi.y > 1023) 
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to y center > 1023"});
      roi_err = true;
   }

   nextroi.width = m_nextROI.w;

   if(nextroi.width < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to width to be < 0"});
      roi_err = true;
   }

   if(nextroi.x + nextroi.width  > 1024) 
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to width such that edge is > 1023"});
      roi_err = true;
   }

   nextroi.height = m_nextROI.h;

   if(nextroi.y + nextroi.height > 1024) 
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to height such that edge is > 1023"});
      roi_err = true;
   }

   if(nextroi.height < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI to height to be < 0"});
      roi_err = true;
   }

   nextroi.x_binning = m_nextROI.bin_x;
   
   if(nextroi.x_binning < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI x binning < 0"});
      roi_err = true;
   }

   nextroi.y_binning = m_nextROI.bin_y;
   
   if(nextroi.y_binning < 0)
   {
      log<software_error>({__FILE__, __LINE__, "can't set ROI y binning < 0"});
      roi_err = true;
   }

   DcamError error;

   if(!roi_err)
   {
      error = Dcam_SetParameterRoisValue( m_cameraHandle, DcamParameter_Rois, &nextrois);   
      if( error != DcamError_None )
      {
         if(powerState() != 1 || powerStateTarget() != 1) return -1;
         std::cerr << DcamEnum2String(DcamEnumeratedType_Error, error) << "\n";
         log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
         state(stateCodes::ERROR);
         return -1;
      }
   }
   
   if(getDcamParameter(readoutStride, DcamParameter_ReadoutStride) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error getting readout stride"});
      state(stateCodes::ERROR);
      return -1;
   }

   if(getDcamParameter(frameStride, DcamParameter_FrameStride) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error getting frame stride"});
      state(stateCodes::ERROR);

      return -1;
   }

   if(getDcamParameter(framesPerReadout, DcamParameter_FramesPerReadout) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error getting frames per readout"});
      state(stateCodes::ERROR);
      return -1;
   }

   if(getDcamParameter( m_frameSize, DcamParameter_FrameSize) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, "Error getting frame size"});
      state(stateCodes::ERROR);
      return -1;
   }

   if(getDcamParameter( pixelBitDepth, DcamParameter_PixelBitDepth) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__,"Error getting pixel bit depth"});
      state(stateCodes::ERROR);
      return -1;
   }
   m_depth = pixelBitDepth;

   const DcamRois* rois;
   error = Dcam_GetParameterRoisValue( m_cameraHandle, DcamParameter_Rois, &rois );
   if( error != DcamError_None )
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1;
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      state(stateCodes::ERROR);
      return -1;
   }
   m_xbinning = rois->roi_array[0].x_binning;
   m_currentROI.bin_x = m_xbinning;
   m_ybinning = rois->roi_array[0].y_binning;
   m_currentROI.bin_y = m_ybinning;
   
   std::cerr << rois->roi_array[0].x << "\n";
   std::cerr << (rois->roi_array[0].x-1) << "\n";
   std::cerr << rois->roi_array[0].width << "\n";
   std::cerr << 0.5*( (float) (rois->roi_array[0].width - 1.0)) << "\n";
   

   if(m_currentFlip == fgFlipLR || m_currentFlip == fgFlipUDLR)
   {
      m_currentROI.x = (1023.0-rois->roi_array[0].x) - 0.5*( (float) (rois->roi_array[0].width - 1.0)) ;
      //nextroi.x = ((1023-m_nextROI.x) - 0.5*( (float) m_nextROI.w - 1.0));
   }
   else
   {
      m_currentROI.x = (rois->roi_array[0].x) + 0.5*( (float) (rois->roi_array[0].width - 1.0)) ;
   }

   
   if(m_currentFlip == fgFlipUD || m_currentFlip == fgFlipUDLR)
   {
      m_currentROI.y = (1023.0-rois->roi_array[0].y) - 0.5*( (float) (rois->roi_array[0].height - 1.0)) ;
      //nextroi.y = ((1023 - m_nextROI.y) - 0.5*( (float) m_nextROI.h - 1.0));
   }
   else
   {
      m_currentROI.y = (rois->roi_array[0].y) + 0.5*( (float) (rois->roi_array[0].height - 1.0)) ;
   }



   
   
   m_currentROI.w = rois->roi_array[0].width;
   m_currentROI.h = rois->roi_array[0].height;
   
   m_width  = rois->roi_array[0].width  / rois->roi_array[0].x_binning;
   m_height = rois->roi_array[0].height / rois->roi_array[0].y_binning;
   Dcam_DestroyRois( rois );


   updateIfChanged( m_indiP_roi_x, "current", m_currentROI.x, INDI_OK);
   updateIfChanged( m_indiP_roi_y, "current", m_currentROI.y, INDI_OK);
   updateIfChanged( m_indiP_roi_w, "current", m_currentROI.w, INDI_OK);
   updateIfChanged( m_indiP_roi_h, "current", m_currentROI.h, INDI_OK);
   updateIfChanged( m_indiP_roi_bin_x, "current", m_currentROI.bin_x, INDI_OK);
   updateIfChanged( m_indiP_roi_bin_y, "current", m_currentROI.bin_y, INDI_OK);


   //We also update target to the settable values
   m_nextROI.x = m_currentROI.x;
   m_nextROI.y = m_currentROI.y;
   m_nextROI.w = m_currentROI.w;
   m_nextROI.h = m_currentROI.h;
   m_nextROI.bin_x = m_currentROI.bin_x;
   m_nextROI.bin_y = m_currentROI.bin_y;

   updateIfChanged( m_indiP_roi_x, "target", m_currentROI.x, INDI_OK);
   updateIfChanged( m_indiP_roi_y, "target", m_currentROI.y, INDI_OK);
   updateIfChanged( m_indiP_roi_w, "target", m_currentROI.w, INDI_OK);
   updateIfChanged( m_indiP_roi_h, "target", m_currentROI.h, INDI_OK);
   updateIfChanged( m_indiP_roi_bin_x, "target", m_currentROI.bin_x, INDI_OK);
   updateIfChanged( m_indiP_roi_bin_y, "target", m_currentROI.bin_y, INDI_OK);
   
   
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   // Exposure Time and Frame Rate
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   if(getDcamParameter(m_ReadOutTimeCalculation, DcamParameter_ReadoutTimeCalculation) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1; 
      return log<software_error, -1>({__FILE__, __LINE__, "could not get ReadOutTimeCalculation"});
   }
   std::cerr << "Readout time is: " <<  m_ReadOutTimeCalculation << "\n";
   updateIfChanged( m_indiP_readouttime, "value", m_ReadOutTimeCalculation/1000.0, INDI_OK); // convert from msec to sec

   const DcamRangeConstraint * constraint_array;
   piint constraint_count;
   DcamAdvanced_GetParameterRangeConstraints( m_modelHandle, DcamParameter_ExposureTime, &constraint_array, &constraint_count);

   if(constraint_count != 1)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1; 
      log<text_log>("Constraint count is not 1: " + std::to_string(constraint_count) + " constraints",logPrio::LOG_ERROR);
   }
   else
   {
      m_minExpTime = constraint_array[0].minimum;
      m_maxExpTime = constraint_array[0].maximum;
      m_stepExpTime = constraint_array[0].increment;

      m_indiP_exptime["current"].setMin(m_minExpTime);
      m_indiP_exptime["current"].setMax(m_maxExpTime);
      m_indiP_exptime["current"].setStep(m_stepExpTime);
   
      m_indiP_exptime["target"].setMin(m_minExpTime);
      m_indiP_exptime["target"].setMax(m_maxExpTime);
      m_indiP_exptime["target"].setStep(m_stepExpTime);
   }

   if(m_expTimeSet > 0)
   {
      long intexptime = m_expTimeSet * 1000 * 10000 + 0.5;
      piflt exptime = ((double)intexptime)/10000;
      capExpTime(exptime);
      std::cerr << "Setting exposure time to " << m_expTimeSet << "\n";
      int rv = setDcamParameter(m_modelHandle, DcamParameter_ExposureTime, exptime);

      if(rv < 0)
      {
         if(powerState() != 1 || powerStateTarget() != 1) return -1; 
         return log<software_error, -1>({__FILE__, __LINE__, "Error setting exposure time"});
      }
   }
   
   piflt exptime;
   if(getDcamParameter(exptime, DcamParameter_ExposureTime) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1; 
      return log<software_error,-1>({__FILE__, __LINE__, "Error getting exposure time"});
   }
   else
   {
      capExpTime(exptime);
      m_expTime = exptime/1000.0;
      m_expTimeSet = m_expTime; //At this point it must be true.
      updateIfChanged(m_indiP_exptime, "current", m_expTime, INDI_IDLE);
      updateIfChanged(m_indiP_exptime, "target", m_expTimeSet, INDI_IDLE);
   }

   if(getDcamParameter(m_FrameRateCalculation, DcamParameter_FrameRateCalculation) < 0)
   {
      if(powerState() != 1 || powerStateTarget() != 1) return -1; 
      return log<software_error,-1>({__FILE__, __LINE__, "Error getting frame rate"});
   }
   else
   {
      m_fps = m_FrameRateCalculation;
      updateIfChanged(m_indiP_fps, "current", m_fps, INDI_IDLE);
   }
   std::cerr << "FrameRate is: " <<  m_FrameRateCalculation << "\n";
   
   piint AdcQuality;
   if(getDcamParameter(AdcQuality, DcamParameter_AdcQuality) < 0)
   {
      std::cerr << "could not get AdcQuality\n";
   }
   std::string adcqStr = DcamEnum2String( DcamEnumeratedType_AdcQuality, AdcQuality );
   std::cerr << "AdcQuality is: " << adcqStr << "\n";

   piflt verticalShiftRate;
   if(getDcamParameter(verticalShiftRate, DcamParameter_VerticalShiftRate) < 0)
   {
      std::cerr << "could not get VerticalShiftRate\n";
   }
   std::cerr << "VerticalShiftRate is: " << verticalShiftRate << "\n";

   piflt AdcSpeed;
   if(getDcamParameter(AdcSpeed, DcamParameter_AdcSpeed) < 0)
   {
      std::cerr << "could not get AdcSpeed\n";
   }
   std::cerr << "AdcSpeed is: " << AdcSpeed << "\n";


   std::cerr << "************************************************************\n";
   
   
   piint AdcAnalogGain;
   if(getDcamParameter(AdcAnalogGain, DcamParameter_AdcAnalogGain) < 0)
   {
      std::cerr << "could not get AdcAnalogGain\n";
   }
   std::string adcgStr = DcamEnum2String( DcamEnumeratedType_AdcAnalogGain, AdcAnalogGain );
   std::cerr << "AdcAnalogGain is: " << adcgStr << "\n";

   if(m_readoutSpeedName == "cmos_00_1MHz" || m_readoutSpeedName == "cmos_01MHz")
   {
      m_emGain = 1;
   }
   else
   {
      piint AdcEMGain;
      if(getDcamParameter(AdcEMGain, DcamParameter_AdcEMGain) < 0)
      {
         std::cerr << "could not get AdcEMGain\n";
      }
      m_emGain = AdcEMGain;
   }
   
/*
   std::cerr << "Onlineable:\n";
   pibln onlineable;
   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_ReadoutControlMode,&onlineable);
   std::cerr << "ReadoutControlMode: " << onlineable << "\n"; //0

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_AdcQuality,&onlineable);
   std::cerr << "AdcQuality: " << onlineable << "\n"; //0

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_AdcAnalogGain,&onlineable);
   std::cerr << "AdcAnalogGain: " << onlineable << "\n"; //1

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_DisableCoolingFan,&onlineable);
   std::cerr << "DisableCoolingFan: " << onlineable << "\n";//0

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_SensorTemperatureSetPoint,&onlineable);
   std::cerr << "SensorTemperatureSetPoint: " << onlineable << "\n"; //0

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_AdcEMGain,&onlineable);
   std::cerr << "AdcEMGain: " << onlineable << "\n"; //1

   Dcam_CanSetParameterOnline(m_modelHandle, DcamParameter_FrameRateCalculation,&onlineable);
   std::cerr << "FrameRateCalculation: " << onlineable << "\n"; //0

   std::cerr << "************************************************************\n";
*/

   //If not previously allocated, allocate a nice big buffer to play with
   pi64s newbuffsz = framesPerReadout*readoutStride*10; //Save room for 10 frames
   if( newbuffsz >  m_acqBuff.memory_size)
   {
      if(m_acqBuff.memory)
      {
         std::cerr << "Clearing\n";
         free(m_acqBuff.memory);
         m_acqBuff.memory = NULL;
         DcamAdvanced_SetAcquisitionBuffer(m_cameraHandle, NULL);
      }

      m_acqBuff.memory_size = newbuffsz;
      std::cerr << "m_acqBuff.memory_size: " << m_acqBuff.memory_size << "\n";
      m_acqBuff.memory = malloc(m_acqBuff.memory_size);

      error = DcamAdvanced_SetAcquisitionBuffer(m_cameraHandle, &m_acqBuff);
      if(error != DcamError_None)
      {
         log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
         state(stateCodes::ERROR);

         std::cerr << "-->" << DcamEnum2String(DcamEnumeratedType_Error, error) << "\n";
      }
   }

   //Start continuous acquisition
   if(setDcamParameter(DcamParameter_ReadoutCount,(pi64s) 0) < 0)
   {
      log<software_error>({__FILE__, __LINE__, "Error setting readouts=0"});
      state(stateCodes::ERROR);
      return -1;
   }

   recordCamera();
   
   error = Dcam_StartAcquisition(m_cameraHandle);
   if(error != DcamError_None)
   {
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      state(stateCodes::ERROR);

      return -1;
   }

   m_dataType = _DATATYPE_UINT16; //Where does this go?
    
   return 0;

}

inline
float dcamCtrl::fps()
{
   return m_fps;
}

inline
int dcamCtrl::startAcquisition()
{
   return 0;
}

inline
int dcamCtrl::acquireAndCheckValid()
{
   piint camTimeOut = 1000; //1 second keeps us responsive without busy-waiting too much

   DcamAcquisitionStatus status;

   DcamAvailableData available;

   DcamError error;
   error = Dcam_WaitForAcquisitionUpdate(m_cameraHandle, camTimeOut, &available, &status);

   if(error == DcamError_TimeOutOccurred) 
   {
      return 1; //This sends it back to framegrabber to check for reconfig, etc.
   }

   clock_gettime(CLOCK_REALTIME, &m_currImageTimestamp);

   if(error != DcamError_None)
   {
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      state(stateCodes::ERROR);

      return -1;
   }
      
   m_available.initial_readout = available.initial_readout;
   m_available.readout_count = available.readout_count;

   if(m_available.initial_readout == 0)
   {
      return 1;
   }

   //std::cerr << "readout: " << m_available.initial_readout << " " << m_available.readout_count << "\n";

   // camera time stamp
   pibyte *frame = NULL;
   pi64s metadataOffset;

   frame = static_cast<pibyte*>(m_available.initial_readout);
   metadataOffset = (pi64s)frame + m_frameSize;

   pi64s *tmpPtr = reinterpret_cast<pi64s*>(metadataOffset);

   double cam_ts = (double)*tmpPtr/(double)m_tsRes;
   double delta_ts = cam_ts - m_camera_timestamp;

   // check for a frame skip
   if(delta_ts > 1.5 / m_FrameRateCalculation){
      std::cerr << "Skipped frame(s)! (Expected a " << 1000./m_FrameRateCalculation << " ms gap but got " << 1000*delta_ts << " ms)\n";
   }
   // print

   m_camera_timestamp = cam_ts; // update to latest

   //fprintf(m_outfile, "%d %-15.8f\n", m_imageStream->md->cnt0+1, (double)*tmpPtr/(double)m_tsRes);

   return 0;

}

inline
int dcamCtrl::loadImageIntoStream(void * dest)
{
   if( frameGrabber<dcamCtrl>::loadImageIntoStreamCopy(dest, m_available.initial_readout, m_width, m_height, m_typeSize) == nullptr) return -1;

   return 0;
}

inline
int dcamCtrl::reconfig()
{
   ///\todo clean this up.  Just need to wait on acquisition update the first time probably.
   
   DcamError error = Dcam_StopAcquisition(m_cameraHandle);
   if(error != DcamError_None)
   {
      log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
      state(stateCodes::ERROR);

      return -1;
   }

   pibln running = true;

   error = Dcam_IsAcquisitionRunning(m_cameraHandle, &running);

   while(running)
   {
      if(MagAOXAppT::m_powerState == 0) return 0;
      sleep(1);

      error = Dcam_StopAcquisition(m_cameraHandle);

      if(error != DcamError_None)
      {
         log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
         state(stateCodes::ERROR);
         return -1;
      }

      piint camTimeOut = 1000;

      DcamAcquisitionStatus status;

      DcamAvailableData available;

      error = Dcam_WaitForAcquisitionUpdate(m_cameraHandle, camTimeOut, &available, &status);
      if(error != DcamError_None)
      {
         log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
         state(stateCodes::ERROR);
         return -1;
      }

//       if(! status.running )
//       {
//          std::cerr << "Not running \n";
// 
//          std::cerr << "status.running: " << status.running << "\n";
//          std::cerr << "status.errors: " << status.errors << "\n";
//          std::cerr << "CameraFaulted: " << (int)(status.errors & DcamAcquisitionErrorsMask_CameraFaulted) << "\n";
//          std::cerr << "CannectionLost: " << (int)(status.errors & DcamAcquisitionErrorsMask_ConnectionLost) << "\n";
//          std::cerr << "DataLost: " << (int)(status.errors & DcamAcquisitionErrorsMask_DataLost) << "\n";
//          std::cerr << "DataNotArriving: " << (int)(status.errors & DcamAcquisitionErrorsMask_DataNotArriving) << "\n";
//          std::cerr << "None: " << (int)(status.errors & DcamAcquisitionErrorsMask_None) << "\n";
//          std::cerr << "ShutterOverheated: " << (int)(status.errors & DcamAcquisitionErrorsMask_ShutterOverheated) << "\n";
//          std::cerr << "status.readout_rate: " << status.readout_rate << "\n";
//       }

      error = Dcam_IsAcquisitionRunning(m_cameraHandle, &running);
      if(error != DcamError_None)
      {
         log<software_error>({__FILE__, __LINE__, 0, error, DcamEnum2String(DcamEnumeratedType_Error, error)});
         state(stateCodes::ERROR);
         return -1;
      }
   }

   return 0;
}



int dcamCtrl::checkRecordTimes()
{
   return telemeter<dcamCtrl>::checkRecordTimes(telem_stdcam());
}
   
int dcamCtrl::recordTelem(const telem_stdcam *)
{
   return recordCamera(true);
}



}//namespace app
} //namespace MagAOX
#endif
