/******************************************************
 *
 * freenect - implementation file
 *
 * (c) copyright 2011/2012 Matthias Kronlachner
 *
 *
 *   institute of electronic music and acoustics (iem)
 *
 ******************************************************
 *
 * license: GNU General Public License v.2
 *
 ******************************************************/

#include "m_pd.h"
#include <string.h>
#include <pthread.h>
#include <stdlib.h>

#include "libfreenect.h"

/* general */

static freenect_context* f_ctx;
static freenect_device* f_dev;


typedef struct _freenect
{
  t_object x_obj;
	
  t_outlet*x_infoout;
        
} t_freenect;

/* ------------------------ freenect ----------------------------- */ 

static t_class *freenect_class;


/*--------------------------------------------------------------------
 * freenect_bang : print info
 *--------------------------------------------------------------------*/

static void freenect_bang(t_freenect*x) {
	post ("\n::freenect status::");
  
	struct freenect_device_attributes *devAttrib = malloc(sizeof *devAttrib);
	int nr_devices = freenect_list_device_attributes(f_ctx, &devAttrib);
  post ("Number of devices found: %d", nr_devices);
	
	// display serial numbers
	const char* id;
	int i = 0;
	for(i=0; i < nr_devices; i++){
		id = devAttrib->camera_serial;
		devAttrib = devAttrib->next;
		post ("Device %d serial: %s", i, id);
	}
	freenect_free_device_attributes(devAttrib);
	
	int ret=freenect_supported_subdevices();
  
	
	if (ret & (1 << 0))
	{
		post ("libfreenect supports FREENECT_DEVICE_MOTOR (%i)", ret);
	}
	if (ret & (1 << 1))
	{
		post ("libfreenect supports FREENECT_DEVICE_CAMERA (%i)", ret);
	}
	if (ret & (1 << 2))
	{
		post ("libfreenect supports FREENECT_DEVICE_AUDIO (%i)", ret);
	}
}

/*--------------------------------------------------------------------
 * freenect_accel : output accelerometer
 *--------------------------------------------------------------------*/
 
static void freenect_accel(t_freenect*x) {
	freenect_raw_tilt_state* state;
	freenect_update_tilt_state(f_dev);
	state = freenect_get_tilt_state(f_dev);
	double dx,dy,dz;
	freenect_get_mks_accel(state, &dx, &dy, &dz);
	
	t_atom ap[4];
  SETFLOAT(ap+0, dx);
  SETFLOAT(ap+1, dy);
  SETFLOAT(ap+2, dz);
	
  outlet_anything(x->x_infoout, gensym("accel"), 3, ap);
  t_atom ap2[1];
  SETFLOAT(ap2, state->tilt_angle);
  outlet_anything(x->x_infoout, gensym("tilt_angle"), 1, ap2);
}

/*--------------------------------------------------------------------
 * freenect_angle : set angle/motor
 *--------------------------------------------------------------------*/
static void freenect_angle(t_freenect*x, t_float f){
	if(f<-30.0)
	{
		f=-30.0;
	} else if (f>30.0)
	{
		f=30.0;
	}
	freenect_set_tilt_degs(f_dev,f);
}

/*--------------------------------------------------------------------
 * freenect_led : set led
 *--------------------------------------------------------------------*/
static void freenect_led(t_freenect*x, t_float led){
  if ( ( (int)led>=0 ) && ( (int)led<=5 ) )
	{
		if ((int)led == 1) {
			freenect_set_led(f_dev,LED_GREEN);
		}
		if ((int)led == 2) {
			freenect_set_led(f_dev,LED_RED);
		}
		if ((int)led == 3) {
			freenect_set_led(f_dev,LED_YELLOW);
		}
		if ((int)led == 4) {
			freenect_set_led(f_dev,LED_BLINK_GREEN);
		}
		if ((int)led == 5) {
			freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
		}
		if ((int)led == 0) {
			freenect_set_led(f_dev,LED_OFF);
		}
	}
}

/*--------------------------------------------------------------------
 * freenect_free : destructor
 *--------------------------------------------------------------------*/
static void freenect_free(t_freenect*x){
	
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	
	if(x->x_infoout)outlet_free(x->x_infoout); x->x_infoout=NULL;
}

static void *freenect_new(t_symbol *s,int argc, t_atom *argv){
  t_freenect *x = (t_freenect *)pd_new(freenect_class);

	post("freenect 0.1 - 2011/12 by Matthias Kronlachner");
	
	if (freenect_init(&f_ctx, NULL) < 0) {
		post("freenect_init() failed\n");
	}
	
	freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR); // LOW LOGLEVEL
	//freenect_set_log_level(f_ctx, FREENECT_LOG_SPEW); // log almost everything

  //struct freenect_device_attributes * devAttrib;
	struct freenect_device_attributes *devAttrib = malloc(sizeof *devAttrib);
	
	int nr_devices = freenect_list_device_attributes(f_ctx, &devAttrib);
  post ("Number of devices found: %d", nr_devices);
	
	// display serial numbers
	const char* id;
	int i = 0;
	for(i=0; i < nr_devices; i++){
		id = devAttrib->camera_serial;
		devAttrib = devAttrib->next;
		post ("Device %d serial: %s", i, id);
	}
	freenect_free_device_attributes(devAttrib);
	
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_MOTOR);

	int openBySerial=0;
	int kinect_dev_nr = 0;
	t_symbol *serial;
	
	if (argc >= 1)
	{
		const char* test = "float";
		
		serial=atom_getsymbol(argv);
		
		if (!strncmp(serial->s_name,"float", 5))
		{
			kinect_dev_nr = (int)atom_getint(argv);
			openBySerial=0;
		} else {
			post ("test: %s", (char*)serial->s_name);
			openBySerial=1;
		}
	}
	
	if (openBySerial == 0)
	{
		verbose(1, "trying to open Kinect device nr %i...", (int)kinect_dev_nr);
		if (freenect_open_device(f_ctx, &f_dev, kinect_dev_nr) < 0) {
			post("ERROR: Could not open Kinect Nr %i !", kinect_dev_nr);
			return(NULL);
		} else
			post("Kinect Nr %d opened", kinect_dev_nr);
	}
	
	// OPEN KINECT BY SERIAL
	if (openBySerial == 1)
	{
		post("trying to open Kinect with serial %s...", (char*)serial->s_name);
		if (freenect_open_device_by_camera_serial(f_ctx, &f_dev, (char*)serial->s_name) < 0) {
			post("ERROR: Could not open Kinect with serial %s !", (char*)serial->s_name);
			return(NULL);
		} else
			post("Kinect with serial %s opened!", (char*)serial->s_name);
	}


	freenect_set_user(f_dev, x);
  
  x->x_infoout=outlet_new(&x->x_obj, NULL);

  return (x);
}

void freenect_setup(void){

  freenect_class = class_new(gensym("freenect"), 
		(t_newmethod)freenect_new, 
			0, sizeof(t_freenect), 
			CLASS_DEFAULT,
			A_GIMME, 0);

	class_addmethod(freenect_class, (t_method)freenect_angle, gensym("angle"), A_FLOAT, 0);
	class_addmethod(freenect_class, (t_method)freenect_led, gensym("led"), A_FLOAT, 0);
	class_addmethod(freenect_class, (t_method)freenect_accel, gensym("accel"), 0);
	class_addbang(freenect_class, (t_method)freenect_bang);
}
