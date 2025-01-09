// Decode function to extract temperature and pressure from the payload
// fPort is the LoRaWAN port number
// bytes is the payload as received from the device
// variables are device variables (not used here but can be added as needed)
function Decode(fPort, bytes, variables) {
    var decoded = {};

    // Ensure that the payload contains the expected 2 bytes for temperature
    if (bytes.length >= 4) {
        // Decode the temperature (using high and low bytes from the payload)
        var tempHighByte = bytes[0];
        var tempLowByte = bytes[1];

        // Reconstruct the 16-bit temperature value (assuming it's a signed integer)
        var temperature = (tempHighByte << 8) | (tempLowByte & 0xFF);
        if (temperature > 32767) {
            temperature -= 65536;  // Handle negative temperature for signed values
        }
        
        decoded.temperature = temperature; // Store the decoded temperature value
      
      	var pressHighByte = bytes[2];
        var pressLowByte = bytes[3];
        var pressure_hpa = (pressHighByte << 8) | pressLowByte;
        //if (pressure > 32767) {
            //pressure -= 65536; // Handle negative values
        //}
        decoded.pressure_hpa = pressure_hpa;

    } else {
        decoded.error = "Invalid byte array length"; // Handle errors if not enough data is received
    }

    return decoded;
}
