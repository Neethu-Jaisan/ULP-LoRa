// Decode function to extract temperature and pressure from the payload
// fPort is the LoRaWAN port number
// bytes is the payload as received from the device
// variables are device variables (not used here but can be added as needed)
function Decode(fPort, bytes, variables) {
    var decoded = {};
	decoded.myValue= (bytes[0]<<8)+bytes[1];
    // Ensure that the payload contains the expected 2 bytes for temperature  
    return decoded;
}
