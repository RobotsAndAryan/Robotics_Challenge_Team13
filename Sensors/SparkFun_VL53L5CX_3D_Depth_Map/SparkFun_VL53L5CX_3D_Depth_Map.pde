import processing.serial.*;

Serial port;
String buff = "";
int[] rawDepths = new int[64];
float[] smoothDepths = new float[64];

int upCols = 29; 
int upRows = 29;
int scale = 25; 
float[][] terrain = new float[upCols][upRows];

float xPress = 0; 
float yPress = 0;

float xRotOffset = 0; 
float xRotPos = 0; 
float zRotOffset = 0; 
float zRotPos = 0; 
float scaleOffset = .5; 

void setup(){
  size(700,700, P3D); 
  
  port = new Serial(this, "/dev/ttyACM0", 115200); 
  port.bufferUntil(10); 
   
  for(int i = 0; i < 64; i++){
    smoothDepths[i] = 0; 
    rawDepths[i] = 0;
  }
}

void draw(){
  colorMode(HSB); 
  lights(); 
  noStroke(); 
  smooth(); 
  background(0); 
  
  translate(width/2,height/2);
  rotateX(PI/3-(xRotOffset+xRotPos));
  rotateZ(0-zRotOffset-zRotPos);
  scale(scaleOffset);
  translate(-width/2, -height/2);

  // smoothing filter (noise reduction)
  for(int i=0; i<64; i++){
    smoothDepths[i] = (smoothDepths[i] * 0.7) + (rawDepths[i] * 0.3);
  }

  // put into normal 8x8 first
  float[][] grid = new float[8][8];
  for(int y=0; y<8; y++){
    for(int x=0; x<8; x++){
      grid[x][y] = smoothDepths[x+y*8] / 10.0;
    }
  }
  
  // bilinear interpolation (extrapolating)
  for(int y=0; y<upRows; y++){
    for(int x=0; x<upCols; x++){
      float gx = x / 4.0;
      float gy = y / 4.0;
      int gxi = int(gx);
      int gyi = int(gy);
      
      // keep it in bounds
      if(gxi >= 7) gxi = 6;
      if(gyi >= 7) gyi = 6;
      
      float dx = gx - gxi;
      float dy = gy - gyi;
      
      float top = grid[gxi][gyi] * (1-dx) + grid[gxi+1][gyi] * dx;
      float bot = grid[gxi][gyi+1] * (1-dx) + grid[gxi+1][gyi+1] * dx;
      
      terrain[x][y] = top * (1-dy) + bot * dy;
    }
  }
  
  // draw the upscaled mesh
  for(int y=0; y<upRows-1; y++){
    beginShape(TRIANGLE_STRIP);
    for(int x=0; x<upCols; x++){
      fill(map(terrain[x][y],0,400,255,0),255,255);
      vertex(x*scale, y*scale, terrain[x][y]);
      vertex(x*scale, (y+1)*scale, terrain[x][y+1]);
    }
    endShape();
  }
}

void serialEvent(Serial p){ 
  buff = (port.readString());
  if (buff != null) {
    buff = buff.trim(); 
    if (buff.length() > 0) {
      int[] temp = int(split(buff, ',')); 
      if(temp.length >= 64) {
        for(int i=0; i<64; i++){
          rawDepths[i] = temp[i];
        }
      }
    }
  }
}

void mousePressed() {
  xPress = mouseX; 
  yPress = mouseY; 
}

void mouseDragged() {
  xRotOffset = (mouseY-yPress)/100; 
  zRotOffset = (mouseX-xPress)/100; 
}

void mouseReleased() {
  xRotPos += xRotOffset;
  xRotOffset = 0;
  zRotPos += zRotOffset;
  zRotOffset = 0;
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scaleOffset += e/10.0;
}
