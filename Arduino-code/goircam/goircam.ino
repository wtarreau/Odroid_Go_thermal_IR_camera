#include <odroid_go.h>
#include "sensors/Wire.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include "BluetoothSerial.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define TA_SHIFT 8 //Default shift for MLX90640 in open air
#define BSWAP16(x) ((((x) & 0xFF) << 8) | ((x) >> 8))

const byte MLX90640_address = 0x33;
static float mlx90640To[768];
paramsMLX90640 mlx90640;

BluetoothSerial serialBT;

const char *rates[4]={ " .5Hz ", " 1 Hz ", " 2 Hz ", " 4 Hz " };
// temp scales: auto, outdoor winter, outdoor summer, indoor, body, water, full
const char *scales[]={ " auto ", "-2>15 ", " 0>30 ", "15>35 ", "25>42 ", " 0>100", " full " };

bool dooverlay=true,saved=false,havesd=false;
int boxx=16,boxy=12;
long gottime,firstsave=-1;
int refresh=3; // 4 Hz by default
int scale=0; // auto by default
int newscale=1;
float mn=-40,mx=300;


void debi(const char *pfx, int i, const char *sfx)
{
  Serial.print(pfx);
  Serial.print(i);
  Serial.println(sfx);
}

void debf(const char *pfx, float f, const char *sfx)
{
  Serial.print(pfx);
  Serial.print(f);
  Serial.println(sfx);
}

void setup()
{
  Serial.begin(115200); // MUST BE BEFORE GO.BEGIN()!!!!!
  GO.begin();
  delay(500);
  Serial.println("Booting...");
  serialBT.begin("GO IR Camera");

  // turn speaker off
  GO.Speaker.setVolume(0);
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);
  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);

  Wire.begin(15,4);
  Wire.setClock(400000);
  Serial.println("Starting...");
  Serial.println();
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");
  MLX90640_SetRefreshRate(MLX90640_address,refresh);
  MLX90640_SetResolution(MLX90640_address, 0);

  /* change the frequency after EEPROM was read */
  Wire.setClock(800000);

  if(SD.begin()) havesd=true;
  else Serial.println("Card Mount Failed");
  GO.lcd.setTextFont(4);
  GO.lcd.setTextColor(WHITE);
  GO.lcd.setTextDatum(MC_DATUM);
//  GO.lcd.setCursor(160,120);
  GO.lcd.drawString("IR Camera v1.1",160,120);
  delay(100);
  getirframe();
  drawtodisplay(true, 0, 23);
  Serial.println("Setup done");
}

void loop()
{
  char inbyte;
  if(serialBT.available()>0)
  {
    inbyte=serialBT.read();
    switch(inbyte)
    {
      case 'd': getirframe();
                sendtoserialtext();
                break;
      case 'i': getirframe();
                drawtodisplay(true, 0, 23);
                break;
      case 't': sendtoserialtext();
                break;
    }
  }
  GO.update();
  if(GO.BtnA.isPressed()==1)
  {
    getirframe();
    drawtodisplay(true, 0, 23);
    newscale=0;
  }
  else
  {
    // only change the color scale when button is released
    newscale=1;
  }

  if(GO.BtnMenu.isPressed()==1)
  {
    while(GO.BtnMenu.isPressed()==1){GO.update();delay(50);}
    if(dooverlay==true) dooverlay=false;
    else dooverlay=true;
    GO.lcd.clearDisplay();
    drawtodisplay(true, 0, 23);
  }
  if(GO.BtnVolume.isPressed()==1)
  {
    scale=(scale+1)%(sizeof(scales)/sizeof(scales[0]));
    drawtodisplay(true, 0, 23);
  }
  if(GO.BtnSelect.isPressed()==1)
  {
    refresh=(refresh+1)&3;
    MLX90640_SetRefreshRate(MLX90640_address,refresh);
    // refresh period - 20% as per datasheet
    delay(800 >> refresh);
    getirframe();
    drawtodisplay(true, 0, 23);
  }
  if(GO.BtnStart.isPressed()==1)
  {
    if(saved==false)
    {
      savetosdcard();
      saved=true;
      drawtodisplay(true, 0, 23);
    }
  }
  delay(50);
  if(GO.JOY_Y.isAxisPressed()==1)
  {
    if(boxy<23 && dooverlay==true)
    {
      while(GO.JOY_Y.isAxisPressed()!=0){GO.update();delay(50);}
      boxy++;
      drawtodisplay(false, 24-boxy, 24-boxy);
      delay(50);
    }
  }
  else if(GO.JOY_Y.isAxisPressed()==2)
  {
    if(boxy>0 && dooverlay==true)
    {
      while(GO.JOY_Y.isAxisPressed()!=0){GO.update();delay(50);}
      boxy--;
      drawtodisplay(false, 22-boxy, 22-boxy);
      delay(50);
    }
  }
  else if(GO.JOY_X.isAxisPressed()==1)
  {
    if(boxx<31 && dooverlay==true)
    {
      while(GO.JOY_X.isAxisPressed()!=0){GO.update();delay(50);}
      boxx++;
      drawtodisplay(false, 23-boxy, 23-boxy);
      delay(50);
    }
  }
  else if(GO.JOY_X.isAxisPressed()==2)
  {
    if(boxx>0 && dooverlay==true)
    {
      while(GO.JOY_X.isAxisPressed()!=0){GO.update();delay(50);}
      boxx--;
      drawtodisplay(false, 23-boxy, 23-boxy);
      delay(50);
    }
  }
}

/* map an intensity from 0 to 255 to an RGB value using the NASA/IPAC colors using 6 equal steps */
uint16_t intensity_to_rgb(int16_t col)
{
  int16_t r,g,b;

  /* R */
  if (col < 32)
    r = 0;
  else if (col < 160)
    r = (col - 32) * 2;
  else
    r = 255;

  /* G */
  if (col < 96)
    g = 192 - col*2;
  else if (col < 139)
    g = 0;
  else if (col < 224)
    g = (col - 139) * 3;
  else
    g = 255;

  /* B */
  if (col < 32)
    b = 255;
  else if (col < 160)
    b = 255 - (col - 32) * 2;
  else if (col < 224)
    b = 0;
  else
    b = (col - 224) * 8;

  /* normally doesn't happen */
  if (r < 0) r = 0; else if (r > 255) r = 255;
  if (g < 0) g = 0; else if (g > 255) g = 255;
  if (b < 0) b = 0; else if (b > 255) b = 255;

  return GO.lcd.color565(r,g,b);
}

static inline float getpix(uint16_t x, uint16_t y)
{
  return mlx90640To[y*32 + x];
}

/* redraws display, optionally from sensor line <from> to line <to> if <cls> is
 * false.
 */
void drawtodisplay(bool cls, uint16_t from, uint16_t to)
{
  uint16_t c,x,y,ix,iy,px,py,ind,col,cnt;
  uint16_t xw=10,yw=10,xoff=0,yoff=0;
  float factor;
  float mid,val,surround;

  Serial.println("enter draw");

  if(dooverlay==true)
  {
    xw=7;
    yw=7;
    xoff=0;
    yoff=0;
  }

  factor = 1.0 / (xw * yw);

  if(newscale)
  {
    mn=300;
    mx=-40;
    for(c=0;c<768;c++)
    {
      if(mlx90640To[c]>mx) mx=mlx90640To[c];
      if(mlx90640To[c]<mn) mn=mlx90640To[c];
    }
  }

  mid=mlx90640To[((23-boxy)*32)+boxx];
  if (mid<-40) mid=-40;
  if (mid>300) mid=300;
  if(mn<-40) mn=-40;
  if(mx>300) mx=300;

  switch (scale)
  {
    case 1: mn=-2; mx=15; break;
    case 2: mn=0; mx=30; break;
    case 3: mn=15; mx=35; break;
    case 4: mn=25; mx=42; break;
    case 5: mn=0; mx=100; break;
    case 6: mn=-40; mx=300; break;
  }

  if (cls) {
    GO.lcd.clearDisplay();
    from = 0;
    to = 23;
  }
  else {
    if (from > 23)
      from = 23;
    if (to > 23)
      to = 23;
  }

  /* Let's iterate over image lines (0..23). Each of these is repeated, but
   * we precompute the positions and ratios which do no change within a line.
   * For each of these lines we do the same for X. The summed value is always
   * multiplied by (xw) and by (yw) since rx0+rx1=xw and ry0+ry1=yw so we
   * have to correct it.
   */
  y = from * yw;
  for (iy = from; iy <= to; iy++) {
    uint16_t frame[320];

    for (py = 0; py < yw; py++, y++) {
      uint16_t y0, y1, ry0, ry1;

      ry1 = py;
      ry0 = yw - ry1;

      y0 = iy;
      y1 = (iy < 23) ? iy+1 : 23;

      for (x = ix = 0; ix < 32; ix++) {
	for (px = 0; px < xw; px++, x++) {
	  uint16_t x0, x1, rx0, rx1;

	  rx1 = px;
	  rx0 = xw - rx1;

	  x0 = ix;
	  x1 = (ix < 31) ? ix+1 : 31;

	  val = (getpix(x0, y0) * (float)rx0 + getpix(x1, y0) * (float)rx1) * (float)ry0 +
	        (getpix(x0, y1) * (float)rx0 + getpix(x1, y1) * (float)rx1) * (float)ry1;
	  val *= factor;

	  // map temp to 0..255
	  col=int(map(int(val*1000),int(mn*1000),int(mx*1000),0,255));
	  col=intensity_to_rgb(col);
	  frame[x] = BSWAP16(col);
	}
      }
      /* draw the line at once. Note that screen and sensor Y coordinates are
       * opposed.
       */
      GO.lcd.pushRect(xoff, yoff+yw+24*yw-y-1, x, 1, frame);
    }
  }

  if (dooverlay) {
    GO.lcd.drawRect(xoff+(boxx*xw),yoff+((boxy+1)*yw),xw,yw,GREEN);

    //if(cls==false)
    //  GO.lcd.fillRect(xoff+(32*xw)+5,0,320-(xoff+(32*xw)),210,BLACK);

    GO.lcd.setTextFont(2);
    GO.lcd.setTextSize(1);
    GO.lcd.setTextDatum(ML_DATUM);
    GO.lcd.setTextColor(WHITE,BLACK);
    GO.lcd.drawFloat(mx,3,255,yoff+15);
    GO.lcd.setTextColor(BLUE,BLACK);
    GO.lcd.drawFloat(mn,3,255,yoff+(24*yw)-2);
    GO.lcd.setTextColor(GREEN,BLACK);
    GO.lcd.drawFloat(mid,3,255,yoff+((24*yw)/2)+yw);

    if (cls) {
      for (y=yoff+7; y<yoff+24*yw+7; y++) {
	col=map(y,yoff+7,yoff+24*yw+7,255,0);
	col=intensity_to_rgb(col);
	GO.lcd.drawLine(235,y,245,y,col);
      }
      GO.lcd.drawRect(234,yoff+7,12,24*yw,WHITE);

      // Draw button labels
      GO.lcd.setTextFont(2);
      GO.lcd.setTextSize(1);
      GO.lcd.setTextColor(BLACK,WHITE);
      GO.lcd.setTextDatum(ML_DATUM);
      GO.lcd.drawString(" ZOOM ",0,230);
      GO.lcd.setTextDatum(MC_DATUM);
      GO.lcd.drawString(scales[scale],100,230);
      GO.lcd.setTextDatum(MC_DATUM);
      GO.lcd.drawString(rates[refresh],220,230);
      if (!saved && havesd) {
	GO.lcd.setTextDatum(MR_DATUM);
	GO.lcd.drawString(" SAVE  ",320,230);
      }
    }
  }
  Serial.println("leave draw");
}

void getirframe()
{
  /* We cannot choose to receive two distinct frames, they're often random,
   * so we're reading one subpage into one frame, then we perform a few
   * retries to try to get the second one.
   */
  uint16_t mlx90640Frame[2][834];
  int status;
  int frm, pages;

  Serial.println("TriggerMeasurement...");
  //MLX90640_TriggerMeasurement(MLX90640_address);
  //Serial.println("SynchFrame...");
  //MLX90640_SynchFrame(MLX90640_address);
  //Serial.println("GetFrameData0...");

  // we don't choose what page the sensor sends us first. The sensor really
  // takes two different pictures, and if they're too far apart, some checker
  // pattern appears. The sensor continues to sample while we read so we can
  // often retrieve the same subpage twice. Thus we're trying to get two
  // consecutive pages and stop once we have one of each.
  pages = 0; // binary 00, 01, 10, 11
  for (frm = 0; pages < 3 && frm < 8; frm++) {
    // status is the page number (0/1) or <0 for errors.
    debi("Retrieving subpage ", pages & 1, "...");
    status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame[pages & 1]);
    debi("Status = ", status, (status >= 0) ? "(ok)" : "(error)");
    if (status < 0)
      return;

    status = !!status;
    debi("Got page ", status, "");

    // not the expected page, resync and try again.
    if ((pages & 1) != status)
      memcpy(mlx90640Frame[status], mlx90640Frame[pages & 1], sizeof(*mlx90640Frame));

    pages |= 1 << status;
  }

  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    float vdd = MLX90640_GetVdd(mlx90640Frame[x], &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame[x], &mlx90640);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    debf(x ? "vdd1 = " : "vdd0 = ", vdd, "");
    debf(x ? "ambient1 = " : "ambient0 = ", tr, "");
    debi("CalculateTo", x, "...");
    MLX90640_CalculateTo(mlx90640Frame[x], &mlx90640, emissivity, tr, mlx90640To);
    Serial.println("Done");
  }
  gottime=millis();
  boxx=16,boxy=12;
  saved=false;
}

void sendtoserialtext()
{
  int x,y,ind,val;
  if(firstsave==-1)
  {
    firstsave=millis();
    serialBT.print("0");
  }
  else serialBT.print(gottime-firstsave,DEC);
  serialBT.print(',');
  for(y=0;y<24;y++)
  {
    for(x=0;x<32;x++)
    {
      ind=(y*32)+x;
      val=int((mlx90640To[ind]+50)*100);
      serialBT.print(val,DEC);
      if(x==31 && y==23) serialBT.println("");
      else serialBT.print(',');
    }
  }
}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

void savetosdcard()
{
  int x,y,ind,val;
  File file=SD.open("/goircam.csv",FILE_APPEND);
  if(!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.print(gottime,DEC);
  file.print(',');
  for(y=0;y<24;y++)
  {
    for(x=0;x<32;x++)
    {
      ind=(y*32)+x;
      val=int((mlx90640To[ind]+50)*100);
      file.print(val,DEC);
      if(!(x==31 && y==23)) file.print(',');
    }
  }
  file.println();
  file.close();
}
