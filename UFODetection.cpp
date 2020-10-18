#include <vector>
#include <iostream>
#include <cmath>
#define pdd pair<double, double>
using namespace std;
//describes position in cartesian coordinate system
class Position {
    public:
        Position(double x, double y)
            :x(x)
            ,y(y) {}
        
        double x; // in meter
        double y; // in meter
};

//radar data input. azimuth measured clockwise from the north. 
class RadarSingleData {
    public:
        RadarSingleData(double azimuth, double distance)
            :azimuth(azimuth)
            ,distance(distance) {}
        
        double azimuth; // in degree
        double distance; // in meter
};

class radar_line{
	public:
	double start_x;
	double end_x;
	double start_y;
	double end_y;
      };

pdd line_intersection(pdd A, pdd B, pdd C, pdd D) 
{ 
    // Line AB represented as a1x + b1y = c1 
    double a1 = B.second - A.second; 
    double b1 = A.first - B.first; 
    double c1 = a1*(A.first) + b1*(A.second); 
  
    // Line CD represented as a2x + b2y = c2 
    double a2 = D.second - C.second; 
    double b2 = C.first - D.first; 
    double c2 = a2*(C.first)+ b2*(C.second); 
  
    double determinant = a1*b2 - a2*b1; 
  
    if (determinant == 0) 
    { 
        // The lines are parallel. This is simplified 
        // by returning a pair of FLT_MAX 
		cout <<"Error";
    } 
    else
    { 
        double x = (b2*c1 - b1*c2)/determinant; 
        double y = (a1*c2 - a2*c1)/determinant; 
        return make_pair(x, y); 
    } 
} 

//function using the RadarSingleData and Position to calculate the position
vector<Position> compute_ufo_positions(
        vector<RadarSingleData> radar1_data,
        vector<RadarSingleData> radar2_data,
        Position radar2_position) {
    //function takes 3 arguments. radar1_data and radar2_data are objects 
    //of RadarSingleData and made into vectors to hold multiple ufo information
    vector<Position> ufo_positions;//value returned by the function
     
    int size_data = radar1_data.size();//important to allow iterations to be run for the 
    //r_line1 corresponds to position detection by radar 1
    radar_line r_line1;
    //r_line2 corresponds to position detection by radar 2
    radar_line r_line2;
    
    //decleration for exact location coordinates
    double x3;
    double y3;
    //exact size of the vector.
//rearrange radar 2 data to correspond with radar 1
vector<RadarSingleData> radar2_dummy = radar2_data;
radar2_data.clear();
for (int j = 0; j < size_data; j++)
{
    //comparison to ensure corresponding values of radars are correct
        double x1 = radar1_data[j].distance * cos(radar1_data[j].azimuth * M_PI /180);
        double y1 = radar1_data[j].distance * sin(radar1_data[j].azimuth * M_PI /180); 
   
    
    for (int i = 0; i < size_data; i++)
    {
            //radar 2 data
        double x2 = radar2_dummy[i].distance * cos(radar2_dummy[i].azimuth * M_PI /180);
        double y2 = radar2_dummy[i].distance * sin(radar2_dummy[i].azimuth * M_PI /180); 
        //ufo position adjusted relative to radar 1
        double x2_adjusted = x2 + radar2_position.x;
        double y2_adjusted = y2 + radar2_position.y;
        //Euclidean distance
        if (sqrt(pow(x2_adjusted - x1,2)+pow(y2_adjusted-y1,2)) < 250)
            {
                radar2_data[j] = radar2_dummy[i];
            }
        
        
    }
    
}

   for (int i = 0; i < size_data; i++){
       //radar1 data
     
        r_line1.end_x = radar1_data[i].distance * cos(radar1_data[i].azimuth * M_PI /180) + 50*cos(radar1_data[i].azimuth * M_PI /180);
        r_line1.start_x = radar1_data[i].distance * cos(radar1_data[i].azimuth * M_PI /180) - 50*cos(radar1_data[i].azimuth * M_PI /180);
        r_line1.end_y = radar1_data[i].distance * sin(radar1_data[i].azimuth * M_PI /180) + 50*sin(radar1_data[i].azimuth * M_PI /180); 
        r_line1.start_y = radar1_data[i].distance * sin(radar1_data[i].azimuth * M_PI /180)-50*sin(radar1_data[i].azimuth * M_PI /180); 
        //radar 2 data
        
        r_line2.end_x = radar2_data[i].distance * cos(radar2_data[i].azimuth * M_PI /180) + 50*cos(radar2_data[i].azimuth * M_PI /180);
        r_line2.start_x = radar2_data[i].distance * cos(radar2_data[i].azimuth * M_PI /180) - 50*cos(radar2_data[i].azimuth * M_PI /180);
        r_line2.end_y = radar2_data[i].distance * sin(radar2_data[i].azimuth * M_PI /180) + 50*sin(radar2_data[i].azimuth * M_PI /180); 
        r_line2.start_y = radar2_data[i].distance * sin(radar2_data[i].azimuth * M_PI /180)-50*sin(radar2_data[i].azimuth * M_PI /180);       
        //ufo position adjusted relative to radar 1
        r_line2.end_x = r_line2.end_x + radar2_position.x;
    	r_line2.start_x = r_line2.start_x + radar2_position.x;
	    r_line2.end_y = r_line2.end_y + radar2_position.y;
	    r_line2.start_y = r_line2.start_y + radar2_position.y;
   
  
   
       
       //Code as per usual begins
        pdd A = make_pair(r_line1.start_x, r_line1.start_y); 
        pdd B = make_pair(r_line1.end_x, r_line1.end_y); 
        pdd C = make_pair(r_line2.start_x, r_line2.start_y); 
        pdd D = make_pair(r_line2.end_x, r_line2.end_y); 
        
        pdd intersection_point = line_intersection(A, B, C, D);
        x3 = intersection_point.first;
        y3 = intersection_point.second;
        
    
        
    
    //gives output in ternms of x and y coordinates
    ufo_positions.push_back(Position(x3, y3));
       
       
   }
   
    return ufo_positions;
};
                                             

int main() {
    vector<RadarSingleData> radar1_data; //input from radar 1
    vector<RadarSingleData> radar2_data; //input from radar 2
    Position radar2_position = Position(200.0, 2000.0); //position of 0radar 2 relevant to radar 1
    vector<Position> ufo_positions;
    
    
    cout << "test case 1" << endl;
    // no ufo (empty lists)
    ufo_positions = compute_ufo_positions(radar1_data, radar2_data, radar2_position);
    // TODO check ufo_positions is empty
    int vectsize = ufo_positions.size();
    if (vectsize == 0){
    cout<<"There is no incoming UFO"<<endl;
    }
    
    cout << "test case 2" << endl;
    // one ufo
    radar1_data.push_back(RadarSingleData(33.690067525979785, 1400.0));
    radar2_data.push_back(RadarSingleData(-50.19442890773481, 1600.0));
    ufo_positions = compute_ufo_positions(radar1_data, radar2_data, radar2_position);
    // TODO check ufo_positions contains one position at (1200, 800)
    vectsize = ufo_positions.size();
    //gives us size of vector and displays 
    for (int i = 0; i < vectsize; i++ ){
    cout<<vectsize<<" UFO detected at:"<<endl;
        
    cout<<"x = "<<ufo_positions[i].x<<" , y = "<<ufo_positions[i].y<<endl;
    }
    
    cout << "test case 3" << endl;
    // two ufos
    radar1_data.push_back(RadarSingleData(45.0, 1700.0));
    radar2_data.push_back(RadarSingleData(-38.65980825409009, 1300.0));
    ufo_positions = compute_ufo_positions(radar1_data, radar2_data, radar2_position);
    // TODO check ufo_positions contains two positions at (1200, 800) and (1200, 1200)
    vectsize = ufo_positions.size();
    //gives us size of vector and displays 
    cout<<vectsize<<" total UFOs detected. There coordinates are as follows: "<<endl;
    for (int i = 0; i < vectsize; i++ ){
    cout<<i+1<<" UFO detected at:";
        
    cout<<"x = "<<ufo_positions[i].x<<" , y = "<<ufo_positions[i].y<<endl;
    }    
    
    
    cout << "test case 3 bis" << endl;
    //same as before but radar data in different order
    radar1_data.clear();
    radar1_data.push_back(RadarSingleData(33.690067525979785, 1400.0));
    radar1_data.push_back(RadarSingleData(45.0, 1700.0));
    radar2_data.clear();
    radar2_data.push_back(RadarSingleData(-38.65980825409009, 1300.0));
    radar2_data.push_back(RadarSingleData(-50.19442890773481, 1600.0));

    ufo_positions = compute_ufo_positions(radar1_data, radar2_data, radar2_position);

    // TODO check ufo_positions contains two positions at (1200, 800) and (1200, 1200)
    vectsize = ufo_positions.size();
    //gives us size of vector and displays 
    cout<<vectsize<<" total UFOs detected. There coordinates are as follows: "<<endl;
    for (int i = 0; i < vectsize; i++ ){
    cout<<" UFO detected at:"<<endl;
        
    cout<<"x = "<<ufo_positions[i].x<<" , y = "<<ufo_positions[i].y<<endl;
    }    
    
       
    cout << "test case 4" << endl;
    // 11 ufos
    radar1_data.clear();
    radar1_data.push_back(RadarSingleData(20.556045219583464, 1700.0));
    radar1_data.push_back(RadarSingleData(32.005383208083494, 1900.0));
    radar1_data.push_back(RadarSingleData(33.690067525979785, 1400.0));
    radar1_data.push_back(RadarSingleData(34.992020198558656, 2400.0));
    radar1_data.push_back(RadarSingleData(41.185925165709655, 2100.0));
    radar1_data.push_back(RadarSingleData(41.6335393365702, 2400.0));
    radar1_data.push_back(RadarSingleData(45.0, 1700.0));
    radar1_data.push_back(RadarSingleData(48.36646066342981, 2400.0));
    radar1_data.push_back(RadarSingleData(53.13010235415598, 2000.0));
    radar1_data.push_back(RadarSingleData(53.97262661489639, 2700.0));
    radar1_data.push_back(RadarSingleData(59.03624346792648, 2300.0));
    
    radar2_data.clear();
    radar2_data.push_back(RadarSingleData(-50.19442890773481, 1600.0));
    radar2_data.push_back(RadarSingleData(-45.0, 2000.0));
    radar2_data.push_back(RadarSingleData(-38.65980825409009, 1300.0));
    radar2_data.push_back(RadarSingleData(-35.53767779197438, 1700.0));
    radar2_data.push_back(RadarSingleData(-23.19859051364819, 1500.0));
    radar2_data.push_back(RadarSingleData(-21.80140948635181, 1100.0));
    radar2_data.push_back(RadarSingleData(-18.43494882292201, 1900.0));
    radar2_data.push_back(RadarSingleData(-14.036243467926477, 1600.0));
    radar2_data.push_back(RadarSingleData(-8.13010235415598, 1400.0));
    radar2_data.push_back(RadarSingleData(0.0, 1000.0));
    radar2_data.push_back(RadarSingleData(8.13010235415598, 1400.0));
    
    ufo_positions = compute_ufo_positions(radar1_data, radar2_data, radar2_position);
        vectsize = ufo_positions.size();
    //gives us size of vector and displays 
    cout<<vectsize<<" total UFOs detected. There coordinates are as follows: "<<endl;
    for (int i = 0; i < vectsize; i++ ){
    cout<<" UFO detected at:"<<endl;
        
    cout<<"x = "<<ufo_positions[i].x<<" , y = "<<ufo_positions[i].y<<endl;
    }   
    return 0;
}


