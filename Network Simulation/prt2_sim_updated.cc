 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include "ns3/adhoc-aloha-noack-ideal-phy-helper.h"


using namespace ns3;
#include <cmath>
#include <vector>
#include <map>
#include <set>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <sstream>





//Drone Tag to identify packets from same drone
class DroneRIDTag : public Tag
{
public:
    DroneRIDTag() : m_droneId(0) {}

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("DroneIdTag")
            .SetParent<Tag>()
            .AddConstructor<DroneRIDTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    void SetDroneId(uint32_t id)
    {
        m_droneId = id;
    }

    uint32_t GetDroneId() const
    {
        return m_droneId;
    }

    uint32_t GetSerializedSize() const override
    {
        return 4;
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_droneId);
    }

    void Deserialize(TagBuffer i) override
    {
        m_droneId = i.ReadU32();
    }

    void Print(std::ostream &os) const override
    {
        os << "DroneId=" << m_droneId;
    }

private:
    uint32_t m_droneId;
};


//dummy protocol number for debugging which channel delivered the packet
static const uint16_t WIFIchannel = 0; 
static const uint16_t BLEChannel[3] = {37, 38, 39};

void sendPacket(Ptr<NetDevice> dev, uint32_t size, uint32_t droneId, uint16_t protocol) {
    Ptr<Packet> pkt = Create<Packet>(size);
    DroneRIDTag tag;
    tag.SetDroneId(droneId);
    pkt->AddPacketTag(tag);
    dev->Send(pkt, dev->GetBroadcast(), protocol);
}

//Observer callback to log received packets
bool receivePacket(Ptr<NetDevice> dev,
                   Ptr<const Packet> pkt,
                   uint16_t protocol,
                   const Address& sender)
{
    DroneRIDTag tag;
    bool hasTag = pkt->PeekPacketTag(tag);

    if (!hasTag)
    {
        std::cout << Simulator::Now().GetSeconds()
                  << "s observer received corrupted packet"
                  << " channel = " << protocol
                  << std::endl;
        return true;
    }

    std::cout << Simulator::Now().GetSeconds()
              << "s observer received packet"
              << " size=" << pkt->GetSize()
              << " bytes"
              << " from=" << sender
              << " droneId=" << tag.GetDroneId()
              << " channel = " << protocol;


    std::cout << std::endl;

    return true;
}




int main(int argc, char *argv[])
{
    //default parameters
    uint32_t nDrones = 1;
    double observationWindow = 10;
    double observationAreaSize = 500;
    double msgPk1Interval = 1;
    double msgPk2Interval = 3;
    uint32_t msgPk1Size = 150;
    uint32_t msgPk2Size = 175;
    int32_t dronesSpeed = -1;
    double contentionWindow = 0.01;


    //command line arguments
    CommandLine cmd;
    cmd.AddValue("nDrones", "Number of drones in the simulation", nDrones);
    cmd.AddValue("observationWindow", "Duration of the observation window (simulation time) in seconds", observationWindow);
    cmd.AddValue("observationAreaSize", "Size of the observation area in meters", observationAreaSize);
cmd.AddValue("dronesSpeed", "Speed of the drones in meters/seconds", dronesSpeed);
cmd.AddValue("contentionWindow", "Size of the contention window in seconds", contentionWindow);
    cmd.Parse(argc, argv);

    Ptr<UniformRandomVariable> backoffRv = CreateObject<UniformRandomVariable>(); //For random backoff value generator
    


    //Set phsyical channel environment
auto channel = CreateObject<MultiModelSpectrumChannel>();
    auto loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3);
    loss->SetReference(1.0, 40.2);  // free-space ref at 2.44 GHz
       channel->AddPropagationLossModel(loss);
    channel->SetPropagationDelayModel(
        CreateObject<ConstantSpeedPropagationDelayModel>());
        auto fading = CreateObject<NakagamiPropagationLossModel>();
channel->AddPropagationLossModel(fading);




    //Setup the drones
    NodeContainer drones;
    drones.Create(nDrones);
    MobilityHelper droneMobility;
droneMobility.SetPositionAllocator(
    "ns3::RandomBoxPositionAllocator",
    "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=" +
                     std::to_string(observationAreaSize) + "]"),
    "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=" +
                     std::to_string(observationAreaSize) + "]"),
    "Z", StringValue("ns3::UniformRandomVariable[Min=20|Max=120]")
);

//Setup random speed for drones if not set by user
    std::ostringstream sp;
    if (dronesSpeed > 0) {
        sp << "ns3::ConstantRandomVariable[Constant=" << dronesSpeed << "]";
    } else {
        sp << "ns3::UniformRandomVariable[Min=10.0|Max=25.0]";
    }

    droneMobility.SetMobilityModel(
    "ns3::GaussMarkovMobilityModel",
    "Bounds", BoxValue(Box(-10000, 10000, -10000, 10000, 0, 500)), //drones can freely leave the area
    "Alpha", DoubleValue(0.85),
    "MeanVelocity", StringValue(sp.str()),
    "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283185307]"),
    "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
    "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=3.0]"),
    "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.5]"),
    "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.05]")
);


        droneMobility.Install(drones);



    //Set up observer
    NodeContainer observer;
    observer.Create(1);
    MobilityHelper observerMobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (observationAreaSize/2.0, observationAreaSize/2.0, 0.0)); //center of the observation area
    observerMobility.SetPositionAllocator (positionAlloc);
    observerMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    observerMobility.Install(observer);



 // Set up wifi specturm
double wifiTxPowerDbm = 13.0;
std::string wifiMode = "ErpOfdmRate6Mbps";
Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                   StringValue(wifiMode));

SpectrumWifiPhyHelper wifiPhy;
wifiPhy.SetChannel(channel);

wifiPhy.Set("ChannelSettings",
            StringValue("{6, 20, BAND_2_4GHZ, 0}"));

            wifiPhy.Set("TxPowerStart", DoubleValue(wifiTxPowerDbm));
wifiPhy.Set("TxPowerEnd", DoubleValue(wifiTxPowerDbm));
wifiPhy.Set("TxPowerLevels", UintegerValue(1));

wifiPhy.Set("TxGain", DoubleValue(0.0));
wifiPhy.Set("RxGain", DoubleValue(0.0));

wifiPhy.Set("RxNoiseFigure", DoubleValue(7.0));

WifiHelper wifi;
wifi.SetStandard(WIFI_STANDARD_80211g);

wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                             "DataMode", StringValue(wifiMode),
                             "ControlMode", StringValue(wifiMode));

WifiMacHelper wifiMac;

wifiMac.SetType("ns3::AdhocWifiMac");

NetDeviceContainer droneWifiDevices =
    wifi.Install(wifiPhy, wifiMac, drones);

NetDeviceContainer observerWifiDevices =
    wifi.Install(wifiPhy, wifiMac, observer);



    //Simulate BLE advertising channels
    NetDeviceContainer txbleChannels[3];
    NetDeviceContainer rxbleChannels[3];


    static const double centerMHz[3] = {2402.0, 2426.0, 2480.0};
    for (size_t i = 0; i < 3; ++i) {
        Bands bands;
        BandInfo bI;
        bI.fl = (centerMHz[i] - 2.0 / 2.0) * 1e6;
        bI.fc = centerMHz[i] * 1e6;
       bI.fh = (centerMHz[i] + 2.0 / 2.0) * 1e6;
       bands.push_back(bI);
       Ptr<SpectrumModel> sModel = Create<SpectrumModel>(bands);

       //Tx Power
        Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(sModel);
        (*txPsd)[0] = 9.977e-10;  // W/Hz  (+3 dBm over 2 MHz)


        //thermal noise PSD
        Ptr<SpectrumValue> noisePsd = Create<SpectrumValue>(sModel);
        (*noisePsd)[0] = 1.9953e-20;  // W/Hz  (-174 + 7 dB NF)



         //Attach it to the drones using adhocalohaNoACK (one way communication).
        AdhocAlohaNoackIdealPhyHelper phyHelper;
        phyHelper.SetChannel(channel);
        phyHelper.SetTxPowerSpectralDensity(txPsd);
        phyHelper.SetNoisePowerSpectralDensity(noisePsd);
        phyHelper.SetPhyAttribute("Rate", DataRateValue(DataRate("125kbps")));


        txbleChannels[i] = phyHelper.Install(drones);
        rxbleChannels[i] = phyHelper.Install(observer);
    }


//Schedule periodic transmission
for(uint32_t droneIndex = 0; droneIndex < nDrones; droneIndex++) {


    Ptr<Node> node = drones.Get(droneIndex);
double guardOffset =  0.05; // add a small guard time to avoid packet 2 overlapping packet 1;
        Ptr<NetDevice> wifiDev = droneWifiDevices.Get(droneIndex);

        //Randomize initial transmission time
        double msg1InitialPhase = backoffRv->GetValue(0.0, msgPk1Interval);
double msg2InitialPhase = backoffRv->GetValue(0.0, msgPk2Interval);

for(double t = msg1InitialPhase; t < observationWindow; t += msgPk1Interval) {

        double randomOffset = backoffRv->GetValue(0.0, contentionWindow); //blind scheduling with random backoff to avoid synchronization of transmissions


        //send on wifi channel
        Simulator::Schedule(Seconds(t + randomOffset), &sendPacket, wifiDev, msgPk1Size, droneIndex, WIFIchannel); 

        //send on ble channels
    for (uint32_t ch = 0; ch < 3; ch++) {
        Time pktAirTime = Seconds(ch*(msgPk1Size * 8.0) / 125000.0); // calculate packet transmission time based on size and data rate
        Ptr<NetDevice> bleDev = txbleChannels[ch].Get(droneIndex);
        Simulator::Schedule(Seconds(t + randomOffset) + MilliSeconds(2 * ch +1) + pktAirTime, &sendPacket, bleDev, msgPk1Size, droneIndex, BLEChannel[ch]); // send on all channels but stagger them slightly
    }

}

for(double t = msg2InitialPhase; t < observationWindow; t += msgPk2Interval) {

            double randomOffset = backoffRv->GetValue(0.0, contentionWindow); //blind scheduling with random backoff to avoid synchronization of transmissions

            //send on wifi channel
        Simulator::Schedule(Seconds(t + randomOffset+guardOffset), &sendPacket, wifiDev, msgPk2Size, droneIndex, WIFIchannel); 

        //send on ble channels
    for (uint32_t ch = 0; ch < 3; ch++) {
        Time pktAirTime = Seconds(ch*(msgPk2Size * 8.0) / 125000.0); // calculate packet transmission time based on size and data rate
        Ptr<NetDevice> bleDev = txbleChannels[ch].Get(droneIndex);
        Simulator::Schedule(Seconds(t + randomOffset+guardOffset) + MilliSeconds(2 * ch +1) + pktAirTime, &sendPacket, bleDev, msgPk2Size, droneIndex, BLEChannel[ch]); // send on all channels but stagger them slightly
    }

}

}

//Add Observer callback to log received packets
Ptr<NetDevice> observerDev = observerWifiDevices.Get(0);
observerDev->SetReceiveCallback(MakeCallback(&receivePacket));
for (uint32_t ch = 0; ch < 3; ch++) {
    Ptr<NetDevice> bleDev = rxbleChannels[ch].Get(0);
    bleDev->SetReceiveCallback(MakeCallback(&receivePacket));
}








//NetAnimationInterface
AnimationInterface anim("ble-wifi-animation.xml");
anim.EnablePacketMetadata(true);

for (uint32_t i = 0; i < drones.GetN(); ++i)
{
    std::ostringstream label;
    label << "Drone " << i;
    anim.UpdateNodeDescription(drones.Get(i), label.str());
}

anim.UpdateNodeDescription(observer.Get(0), "Observer");
Simulator::Stop(Seconds(observationWindow));
Simulator::Run();
Simulator::Destroy();

return 0;

}