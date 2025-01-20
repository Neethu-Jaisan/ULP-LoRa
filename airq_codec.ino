function decodeUplink(input) {
  return {
    data:Decode(input.fPort, input.bytes, input.variables)
  };
}
function Decode(fPort, bytes, variables) {
  var decoded = {};
  //var i = 0;
  //decoded.dummy_data = bytes[++i];
  
      decoded.Temperature = ((bytes[0]<<8) + (bytes[1]))/100;
      decoded.Humidity = (bytes[2]);
      decoded.dust_density = (bytes[3]<<8) + (bytes[4]);
      decoded.battery_voltage = (bytes[5]);

  return decoded;
}
  
