/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#include <sstream> //stream for io
#include <fstream>
#include <iostream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"

//Needed for file I/O
using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Assignment");

static bool g_verbose = true;
static std::string mac_sa;
static std::string power;
static Ptr<Node> n;

//Used to hold location coordinates
struct coordinates {
	double x;
	double y;

};


// Trilateration method from class notes
std::vector<double> trilateration(double pointA[], double pointB[], double pointC[], double ad, double bd, double cd)
{
   std::vector<double> result;

    double s = (pow(pointC[0], 2) - pow(pointB[0], 2) + pow(pointC[1], 2) - pow(pointB[1], 2) + pow(bd, 2) - pow(cd, 2))/2;
    double t = (pow(pointA[0], 2) - pow(pointB[0], 2) + pow(pointA[1], 2) - pow(pointB[1], 2) + pow(bd, 2) - pow(ad, 2))/2;
    double y = (t*(pointB[0] - pointC[0]) - s*(pointB[0] - pointA[0]))/((pointA[1] - pointB[1])*(pointB[0] - pointC[0]) - (pointC[1] - pointB[1])*(pointB[0] - pointA[0]));
    double x = (y*(pointA[1] - pointB[1]) - t)/(pointB[0] - pointA[0]);

    result.push_back(x);
    result.push_back(y);

   return result;
}


//Distance between two points method
//Used to calculate error between trilaterated and actual positions
//Used to calculate the positions of paintings relative to client node

double Distance2Points(double x1, double y1, double x2, double y2)
{
	double distance;
	double dx, dy;
    dx = x2 - x1;
    dy = y2 - y1;
    distance = sqrt(dx*dx + dy*dy);
    return distance;

}


//VECTORS TO STORE DATA
//this vector stores the distances
static std::vector<double> g_distances;
//this vector stores the power [dbm]
static std::vector<double> g_powers;

//This is where the live data received from client node is stored
//this map stores value pairs [MAC,power], where MAC is a unique value
//e.g. [00:00:00:00:00:01,-96]
static std::map<std::string, std::string> g_mapBeacons;

//Map for AP locations storing coordinates
//Stores MAC and AP coordinates
static std::map<std::string, coordinates> g_mapAPLocations;


static void SetPosition(Ptr<Node> node, Vector position) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	mobility->SetPosition(position);
}

static Vector GetPosition(Ptr<Node> node) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	return mobility->GetPosition();
}

//After setting the scheduler method below, we make the node move at 0.5m every second
static void AdvancePosition(Ptr<Node> node) {
	Vector pos = GetPosition(node);
	pos.x += 0.5;
	pos.y += -0.1;

	//Each time this function runs, it calls the schedule method again to trigger in 1 sec
	//***Use something like below for writing to the map array***
	SetPosition(node, pos);
	Simulator::Schedule(Seconds(1.0), &AdvancePosition, node);
}

//Here we print the position of the node
//We print the lines that appear when we run the simulation
//We add the MAC address and power to be printed too
static void PrintCurrentPosition(Ptr<Node> node)
{

	Vector pos = GetPosition(node);

	NS_LOG_UNCOND ("Current Position: x=" << pos.x << " y=" << pos.y << " MAC SA= "<<mac_sa << " Power: " << power);
	Simulator::Schedule(Seconds(1.0), &PrintCurrentPosition, node);
}

static void AssocTrace(std::string context, Mac48Address address) {

	NS_LOG_UNCOND("Associated with AP");
}


static void WifiPhyRx(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, bool isShortPreamble, double signalDbm, double noiseDbm) {

	Ptr<Packet> p_copy = packet->Copy();

	std::ostringstream  os;
	p_copy->Print(os);
//	std::cout << os.str() ;

	std::string packetinfo = os.str();

//	NS_LOG_UNCOND(p_copy->GetSize() << " - " << packetinfo << signalDbm << " ceva");
	std::string str_mngmt_beacon="MGT_BEACON";
	std::size_t beacon_found = os.str().find(str_mngmt_beacon);

	std::ostringstream strs;
	strs << signalDbm;
	std::string signalDbm_str = strs.str();

//	As this runs, if a beacon is found, the MAC and power is picked up from every beacon that is found
//	and the information is stored in g_mapBeacons.

	if (beacon_found!=std::string::npos)
	{
		//NS_LOG_UNCOND("Beacon Received");
		std::string str_sa="SA=";
		std::size_t sa_found = os.str().find(str_sa);

		std::string str_sa_msc="ff:ff:ff:ff:ff:ff";
		str_sa_msc = os.str().substr(sa_found+3,17);

		mac_sa=str_sa_msc;
		power=signalDbm_str;
		//The mac and power need to be added to the map array

		// TODO: update the g_mapBeacons with the newly received beacon information
		// Need to get g_mapBeacons
// Each time beacon received, mapping mac and power
		g_mapBeacons [mac_sa] = power;

		//NS_LOG_UNCOND("x:" << GetPosition(n).x << ":y:" <<GetPosition(n).y << ":MAC:" << str_sa_msc << ":Power[dBm]:" << signalDbm_str);
	}
}


//Method calculating the position of the moving node
//TODO: Calculate the location of the moving node using Trilateration
void calculateTrilateration () {

	//'it' is a pointer that looks to the first entry in the g_mapBeacons
	//Assigns it to first element in g_mapBeacons

	//First column element in g_mapBeacons (the MAC address) is assigned to amac
	//amac holds MAC of first entry

	//Define variable to hold the element position
	//beaconPower holds power for amac which is received by client node

	//To do trilateration we require 3 beacons to be in range
	if (g_mapBeacons.size()>=3)
	{
		//must have more than 3 items (MAC & Power) in the array
		//NS_LOG_UNCOND(g_mapBeacons.size() << "AP nodes are within reach, can begin trilateration");

		//g_powers is the array holding the tracking.dat powers
		//The indices of each entry correspond to the indices in the g_distances array
		int g_powers_size = g_powers.size(); //The number of items in the array

		//	Min power fix - see below:
		//	Fixes the situation where a reading slightly higher than the last entry in g_powers was received (weakest signal!)
		int min_power = (g_powers_size -1); //last item and weakest signal in g_powers (about -95.8329 Db)
		//	locs represent coordinates for each access point (beacon)
		coordinates loc1, loc2, loc3;
		//	now need to find the distance between elements
		//	Go to g_powers element, find closest matching power and note the index number
		//	Map this index number to the g_distances array to find the corresponding distance for the power
		double searcheddistance1, searcheddistance2, searcheddistance3;

//	First Line
		//	iterate through the g_mapBeacons (the live data being received by beacons!!!)
		std::map<std::string,std::string>::iterator it = g_mapBeacons.begin();

			//	store the MAC address of the first element as amac1
			//	by using a pointer for the first column in this map (MAC!)
		std::string amac1 = it->first;
			//	Store the second column element (Power!) as beaconPower1
		double beaconPower1 = std::atof(it->second.c_str());

			//	Look up g_mapAPLocations for the coordinates of the MAC stored in amac1
			//	Store coordinates of amac1 as loc1
		loc1 = g_mapAPLocations[amac1];

			//	Iterate through g_powers (the power data array created from tracking.dat!)
			//	until the next item in the array is a stronger signal
			//	Set locpower1 as equal to the index of this power
		uint16_t locpower1;

		//For each element in the static g_powers array, iterate until the next power value
		//is less than the received beacon power. Set the locpower equal to the index of this power value
		for (int j=0; j<g_powers_size; j++)
			{
				if (g_powers[j] < beaconPower1)
				{
					locpower1=j;
					break;

				}

				//	Fix for when the received beaconPower1 from amac1 is weaker than the very last element power in g_powers
				// 	IE: If the beaconPower received is less than -95.8329 (last element power in static g_powers array)
				//  Set locpower1 to the index of last element in g_powers
				if (beaconPower1 < g_powers[min_power])
				{
					//Set power to the minimum (-95.8329 DB)
					//The connection with this beacon will terminate soon
					//because the power is too low to maintain a connection with client
					locpower1 = min_power;
				}
			}

		searcheddistance1 = g_distances[locpower1];

			//NS_LOG_UNCOND("AP1:" << searcheddistance1 << "far away");

			//Iterate the pointer - to next line in g_mapBeacons
		it++;


//	Second line of g_mapBeacons - second beacon
		std::string amac2 = it->first;
		double beaconPower2 = std::atof(it->second.c_str());
		loc2 = g_mapAPLocations[amac2];
		uint16_t locpower2;

		for (int k=0; k<g_powers_size; k++)
			{
				if (g_powers[k] < beaconPower2)
				{
					locpower2=k;
					break;
				}

				if (beaconPower2 < g_powers[min_power])
				{
					locpower2 = min_power;
				}
			}
		searcheddistance2 = g_distances[locpower2];

		//NS_LOG_UNCOND("AP2:" << searcheddistance2 << "far away");

		it++;

//	Third line of g_mapBeacons - third and final beacon
		std::string amac3 = it->first;
		double beaconPower3 = std::atof(it->second.c_str());
		loc3 = g_mapAPLocations[amac3];
		uint16_t locpower3;

		for (int l=0; l<g_powers_size; l++)
			{
				if (g_powers[l] < beaconPower3)
				{
					locpower3 = l;
					break;
				}

				if (beaconPower3 < g_powers[min_power])
				{
					locpower3 = min_power;
				}
			}
		searcheddistance3 = g_distances[locpower3];


// TRILATERATION
			std::vector<double> trilateratedResult;
			double pA[] = {loc1.x,loc1.y};
			double pB[] = {loc2.x,loc2.y};
			double pC[] = {loc3.x,loc3.y};
			trilateratedResult = trilateration(pA, pB, pC, searcheddistance1, searcheddistance2, searcheddistance3);

//	Save x and y values for each trilateration result
			double triX = trilateratedResult[0];
			double triY = trilateratedResult[1];

//	Distance between two points using Distance2Points
			double error_margin;
			double point1x = triX;
			double point1y = triY;
			double point2x = GetPosition(n).x;
			double point2y = GetPosition(n).y;
			//Populate constructor
			//Returns distance between trilaterated and actual coordinates
			error_margin = Distance2Points(point1x, point1y, point2x, point2y);

//	Print to file
			std::ofstream myfile;
			myfile.open ("trilateration.dat", ios::out | ios::app );
			myfile <<"[ "<< GetPosition(n).x << ",	" << GetPosition(n).y << "] , [" << trilateratedResult[0] << ", " << trilateratedResult[1] <<"]  Error Margin:  " << error_margin << "\n";

// Console Logging
			NS_LOG_UNCOND("     Locpower1 index: [" << locpower1 <<"] ,    Power: [" << g_powers[locpower1] << "]" );
			NS_LOG_UNCOND("     Ap1: [" << loc1.x << "," << loc1.y <<"]    Distance from Ap1 = " <<searcheddistance1 <<  "m");
			NS_LOG_UNCOND("     Locpower2 index: [" << locpower2 <<"] ,    Power: [" << g_powers[locpower2] << "]" );
			NS_LOG_UNCOND("     Ap2: [" << loc2.x << "," << loc2.y <<"]    Distance from Ap2 = " <<searcheddistance2 <<  "m");
			NS_LOG_UNCOND("     Locpower3 index: [" << locpower3 <<"] ,    Power: [" << g_powers[locpower3] << "]" );
			NS_LOG_UNCOND("     Ap3: [" << loc3.x << "," << loc3.y <<"]    Distance from Ap3 = " <<searcheddistance3 <<  "m");
			NS_LOG_UNCOND("___________________________________________________________");

			// formula
			// see uncommented bit above
			NS_LOG_UNCOND ("     ---        Actual Position:    [" << GetPosition(n).x << ", " << GetPosition(n).y << "]" );
			NS_LOG_UNCOND ("     ---  Trilaterated Position:    [" << trilateratedResult[0] << ", " << trilateratedResult[1] << "]");
			NS_LOG_UNCOND ("     ---           Error Margin:     " << error_margin << "" );
	}
	else
	{
		//print a text saying that you do not have enough information for location detection
		NS_LOG_UNCOND("Not enough beacons received for trilateration.");
	}

//	Execute with scheduler - each time client node advances, call calculateTrilateration method again

	Simulator::Schedule(Seconds(0.5), &calculateTrilateration);

//	Clear the g_mapBeacons
	g_mapBeacons.clear();
}


//	Reads content from file (coordinates, power) and pushes these values into 2 arrays
//	 g_powers
// 	 g_distances.
//	The file that populates these arrays was created from Task1.
//  Distance from a single AP on the same axis on which the client was traveling was recorded, as well
//	as the power received. This data was is used as a look-up table to calculate powers and distances.
//	The indexes of the arrays make it easy to loop through values and find corresponding indexes quickly.
void LoadPowerDistanceMapping (std::string filename)
{
  double distance, power;
  std::ifstream ifTraceFile;
  ifTraceFile.open (filename.c_str (), std::ifstream::in);
  g_distances.clear ();
  g_powers.clear();
  if (!ifTraceFile.good ())
    {
      NS_LOG_UNCOND("Something wrong with the file.");
    }
  while (ifTraceFile.good ())
    {
      ifTraceFile >> distance >> power;

      g_distances.push_back (distance);
      g_powers.push_back(power);
      // NS_LOG_UNCOND(distance << " " << power);
    }
  ifTraceFile.close ();
}


//	MAIN FUNCTION - Start here to configure simulation

int main(int argc, char *argv[]) {
	CommandLine cmd;
	cmd.AddValue("verbose", "Print trace information if true", g_verbose);
	cmd.Parse(argc, argv);

	remove("ass4_tpVsTime.txt");
	remove("ass4_tpVsDistanceToAp.txt");
	remove("ass4_modulation.txt");
	int standard = 2; // 0 = a, 1 = b, 2 = g

	//How long the simulation time is, in seconds
	//Simulation goes faster than real time
	double simulation_time = 370.0;	//g

	//TODO: Call here the LoadPowerDistanceMapping(filename);
	LoadPowerDistanceMapping("./scratch/tracking.dat"); //loading this file

	// enable rts cts all the time.
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("0"));
	// disable fragmentation
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));

	NS_LOG_INFO ("Create nodes.");
	NodeContainer ap; //all AP nodes

	NodeContainer stas; //all station nodes
	stas.Create(1);
	n=stas.Get(0);

	ap.Create(3);

//	Nodes are below - we need to create our own nodes
		//Stas = stations
		//For each node we need to define the communication standard
		//Below is done for wifi - using NS3 standard
		//Can change to 80211a b or c
		//Nosq wifi helpers...
		//Need to define which nodes are access points and which are devices


	NS_LOG_INFO ("Set Standard.");
	WifiHelper wifi = WifiHelper::Default();

	if (standard == 0) {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
	} else if (standard == 1) {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
	} else {
		wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	}

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	wifiPhy.SetChannel(wifiChannel.Create());

	wifiPhy.Set("TxPowerEnd",DoubleValue (-5.0206));
	wifiPhy.Set("TxPowerStart",DoubleValue (-5.0206));

	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	wifi.SetRemoteStationManager("ns3::ArfWifiManager");

//	Give station a name
	Ssid ssid = Ssid("museum");

// 	Setup stas.
	NetDeviceContainer staDevs, apDevs;
	wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
			BooleanValue(false));
	staDevs = wifi.Install(wifiPhy, wifiMac, stas);

//	Setup ap. Stations above, nodes (devices below)
	wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid),
			"EnableBeaconJitter", BooleanValue(true),
			"BeaconInterval", TimeValue (MicroSeconds (102400)));
	apDevs = wifi.Install(wifiPhy, wifiMac, ap);

//	Define where access points and devices are located
	//Need to use Mobility model and install it on station
	//Mobility model has a fixed position

	NS_LOG_INFO ("Set Positions:");
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	mobility.Install(stas);


	mobility.Install(ap);
//	3 Access Points - Beacons

//	Access points in the ap container are distributed in a grid but we don't want this
	mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue(0.0),"MinY", DoubleValue (0.0),
	"DeltaX", DoubleValue (100), "DeltaY", DoubleValue (100), "GridWidth", UintegerValue (4), "LayoutType", StringValue ("RowFirst"));

//	mobility.Install (ap); // If we want the grid, but we don't, so up top

//	CREATE COORDS FOR APs using coordinates struct defined at top
	SetPosition(ap.Get(0), Vector(22.0, 0.0, 0.0));
	coordinates c1;
	c1.x=GetPosition(ap.Get(0)).x;
	c1.y=GetPosition(ap.Get(0)).y;
	g_mapAPLocations["00:00:00:00:00:02"]=c1; //Adds coordinates to map for MAC of beacon

	SetPosition(ap.Get(1), Vector(56.0, 00.0, 0.0));
	coordinates c2;
	c2.x=GetPosition(ap.Get(1)).x;
	c2.y=GetPosition(ap.Get(1)).y;
	g_mapAPLocations["00:00:00:00:00:03"]=c2; //Check MAC Addresses from simulator

	SetPosition(ap.Get(2), Vector(30.0, -24.0, 0.0));
	coordinates c3;
	c3.x=GetPosition(ap.Get(2)).x;
	c3.y=GetPosition(ap.Get(2)).y;
	g_mapAPLocations["00:00:00:00:00:04"]=c3;



// 	CLIENT NODE SETUP
	Vector pos = GetPosition(stas.Get(0));
	pos.x = pos.x - 2;   //change this x coordinate to move the CLIENT node
	pos.y = pos.y -14;  //change this y coordinate to move the CLIENT node
	SetPosition(stas.Get(0), pos);

	//We need to make sure the stacks are installed on each node
	//Here we are defining the Internet stack and assigning IP addresses to each node
	InternetStackHelper stack;
	stack.Install(ap);
	stack.Install(stas);

	NS_LOG_INFO ("Assign IP Addresses:");
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer i = ipv4.Assign(staDevs);
	ipv4.Assign(apDevs);

	//Below, tell the simulator to schedule an event (NS3 is event based) every 1.0 second
	//and execute this on the first node element (0). The event is AdvancePosition (make it move!)
	//Go back up to top and schedule more events at 'static void AdvancePosition!!

	Simulator::Schedule(Seconds(1.0), &AdvancePosition, stas.Get(0));


//	Call the calculateTrilateration() method
	calculateTrilateration();

	Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
				MakeCallback(&AssocTrace));

	Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
					MakeCallback(&WifiPhyRx));

//	Start Simulation
	NS_LOG_INFO ("Run Simulation.");
	Simulator::Stop(Seconds(simulation_time));
	Simulator::Run();
	Simulator::Destroy();
	NS_LOG_INFO ("Done.");

	return 0;
}
