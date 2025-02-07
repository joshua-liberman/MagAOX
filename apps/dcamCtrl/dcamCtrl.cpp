/** \file dcamCtrl.cpp
  * \brief The MagAO-X Princeton Instruments EMCCD camera controller main program.
  *
  * \author Joshua Liberman (jliberman@arizona.edu)
  * 
  * \ingroup dcamCtrl_files
  */


#include "dcamCtrl.hpp"

int main(int argc, char ** argv)
{
   MagAOX::app::dcamCtrl dcam;

   return dcam.main(argc, argv);
}
