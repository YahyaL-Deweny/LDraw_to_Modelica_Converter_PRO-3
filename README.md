# LDraw_to_Modelica_Converter_PRO-3

 The Script provides an automation process for Parsing and Converting: “LDraw” File into “.mo” Format
 to be used as a LeoCAD Plug-in for Simulation Purposes.
 
The code takes an LDR file as an input to produce a modelica code as an output (cutomized only for gears for gears)
  - The name of the LDR file(within script) is changeable and must match the name of the actual file!
  - Also the Dictionary that contains the information about gears, has to cover and to be filled with all the gears that exist in the LDR file first.  
  - Script only handles gears with rotation angels of 90 degrees and its multiples. Rotation of gears has to be made only after being initailly setting the       coordinate axis of the part(gear) parallel to the reference coordinate axis of LeoCAD. 


!!! Disclaimer !!!

The code is finalized but it is fully operable yet. The script main pillar is depends on one chunk of code which contains the logic of interpreting each gear's position and orientation, and there are three cases:
- 1st case: gear surface is parallel to Z-axis (rotates around Y-axis)
- 2nd case: gear surface is parallel to X-axis (rotates around Z-axis)
- 3rd case: gear surface is parallel to Y-axis (rotates around X-axis)
  
simply the logic responsible for any of the previous cases is similar ,however, some variables have to be adjusted to suit each case , at the meantime, only the 1st case is covered and ready for operation. 

