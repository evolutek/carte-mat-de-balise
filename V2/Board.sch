<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.5.2">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="LibBeaconMat" urn="urn:adsk.eagle:library:16498234">
<packages>
<package name="B06B-PASK" urn="urn:adsk.eagle:footprint:11903778/3" library_version="3">
<pad name="P$1" x="5" y="0" drill="0.7" shape="square"/>
<pad name="P$2" x="3" y="0" drill="0.7" shape="square"/>
<pad name="P$3" x="1" y="0" drill="0.7" shape="square"/>
<pad name="P$4" x="-1" y="0" drill="0.7" shape="square"/>
<pad name="P$5" x="-3" y="0" drill="0.7" shape="square"/>
<pad name="P$6" x="-5" y="0" drill="0.7" shape="square"/>
<text x="6" y="0.5" size="1.27" layer="21">1</text>
<text x="-2.81" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.81" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<wire x1="7" y1="2.15" x2="-7" y2="2.15" width="0.127" layer="51"/>
<wire x1="-7" y1="-3.15" x2="7" y2="-3.15" width="0.127" layer="51"/>
<wire x1="7" y1="2.15" x2="7" y2="-3.15" width="0.127" layer="51"/>
<wire x1="-7" y1="-3.15" x2="-7" y2="2.15" width="0.127" layer="51"/>
</package>
<package name="B05B-PASK" urn="urn:adsk.eagle:footprint:11903780/3" library_version="3">
<pad name="P$1" x="4" y="0" drill="0.7" shape="square"/>
<pad name="P$2" x="2" y="0" drill="0.7" shape="square"/>
<pad name="P$3" x="0" y="0" drill="0.7" shape="square"/>
<pad name="P$4" x="-2" y="0" drill="0.7" shape="square"/>
<pad name="P$5" x="-4" y="0" drill="0.7" shape="square"/>
<text x="5" y="0.5" size="1.27" layer="21">1</text>
<text x="-3.81" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<wire x1="6" y1="2.15" x2="-6" y2="2.15" width="0.127" layer="51"/>
<wire x1="-6" y1="-3.15" x2="6" y2="-3.15" width="0.127" layer="51"/>
<wire x1="6" y1="2.15" x2="6" y2="-3.15" width="0.127" layer="51"/>
<wire x1="-6" y1="-3.15" x2="-6" y2="2.15" width="0.127" layer="51"/>
</package>
<package name="DIP787W45P254L3556H508Q28" urn="urn:adsk.eagle:footprint:16498235/1" library_version="1">
<wire x1="3.75" y1="17.78" x2="-3.75" y2="17.78" width="0.127" layer="51"/>
<wire x1="-3.75" y1="17.78" x2="-3.75" y2="-17.78" width="0.127" layer="51"/>
<wire x1="-3.75" y1="-17.78" x2="3.75" y2="-17.78" width="0.127" layer="51"/>
<wire x1="3.75" y1="-17.78" x2="3.75" y2="17.78" width="0.127" layer="51"/>
<wire x1="3.75" y1="17.78" x2="-3.75" y2="17.78" width="0.127" layer="21"/>
<wire x1="3.75" y1="-17.78" x2="-3.75" y2="-17.78" width="0.127" layer="21"/>
<wire x1="-3.75" y1="17.78" x2="-3.75" y2="17.526" width="0.127" layer="21"/>
<wire x1="3.75" y1="17.78" x2="3.75" y2="17.526" width="0.127" layer="21"/>
<wire x1="-3.75" y1="-17.78" x2="-3.75" y2="-17.526" width="0.127" layer="21"/>
<wire x1="3.75" y1="-17.78" x2="3.75" y2="-17.526" width="0.127" layer="21"/>
<wire x1="4.8" y1="18.03" x2="-4.8" y2="18.03" width="0.05" layer="39"/>
<wire x1="-4.8" y1="18.03" x2="-4.8" y2="-18.03" width="0.05" layer="39"/>
<wire x1="-4.8" y1="-18.03" x2="4.8" y2="-18.03" width="0.05" layer="39"/>
<wire x1="4.8" y1="-18.03" x2="4.8" y2="18.03" width="0.05" layer="39"/>
<circle x="-5.71441875" y="16.5443" radius="0.1" width="0.2" layer="21"/>
<circle x="-5.71441875" y="16.5443" radius="0.1" width="0.2" layer="51"/>
<text x="-4.76866875" y="18.4386" size="1.781040625" layer="25">&gt;NAME</text>
<text x="-4.94563125" y="-20.1171" size="1.78091875" layer="27">&gt;VALUE</text>
<pad name="1" x="-3.935" y="16.51" drill="0.82" diameter="1.23" shape="square"/>
<pad name="2" x="-3.935" y="13.97" drill="0.82" diameter="1.23"/>
<pad name="3" x="-3.935" y="11.43" drill="0.82" diameter="1.23"/>
<pad name="4" x="-3.935" y="8.89" drill="0.82" diameter="1.23"/>
<pad name="5" x="-3.935" y="6.35" drill="0.82" diameter="1.23"/>
<pad name="6" x="-3.935" y="3.81" drill="0.82" diameter="1.23"/>
<pad name="7" x="-3.935" y="1.27" drill="0.82" diameter="1.23"/>
<pad name="8" x="-3.935" y="-1.27" drill="0.82" diameter="1.23"/>
<pad name="9" x="-3.935" y="-3.81" drill="0.82" diameter="1.23"/>
<pad name="10" x="-3.935" y="-6.35" drill="0.82" diameter="1.23"/>
<pad name="11" x="-3.935" y="-8.89" drill="0.82" diameter="1.23"/>
<pad name="12" x="-3.935" y="-11.43" drill="0.82" diameter="1.23"/>
<pad name="13" x="-3.935" y="-13.97" drill="0.82" diameter="1.23"/>
<pad name="14" x="-3.935" y="-16.51" drill="0.82" diameter="1.23"/>
<pad name="15" x="3.935" y="-16.51" drill="0.82" diameter="1.23"/>
<pad name="16" x="3.935" y="-13.97" drill="0.82" diameter="1.23"/>
<pad name="17" x="3.935" y="-11.43" drill="0.82" diameter="1.23"/>
<pad name="18" x="3.935" y="-8.89" drill="0.82" diameter="1.23"/>
<pad name="19" x="3.935" y="-6.35" drill="0.82" diameter="1.23"/>
<pad name="20" x="3.935" y="-3.81" drill="0.82" diameter="1.23"/>
<pad name="21" x="3.935" y="-1.27" drill="0.82" diameter="1.23"/>
<pad name="22" x="3.935" y="1.27" drill="0.82" diameter="1.23"/>
<pad name="23" x="3.935" y="3.81" drill="0.82" diameter="1.23"/>
<pad name="24" x="3.935" y="6.35" drill="0.82" diameter="1.23"/>
<pad name="25" x="3.935" y="8.89" drill="0.82" diameter="1.23"/>
<pad name="26" x="3.935" y="11.43" drill="0.82" diameter="1.23"/>
<pad name="27" x="3.935" y="13.97" drill="0.82" diameter="1.23"/>
<pad name="28" x="3.935" y="16.51" drill="0.82" diameter="1.23"/>
</package>
</packages>
<packages3d>
<package3d name="B06B-PASK" urn="urn:adsk.eagle:package:11903788/5" type="model" library_version="3">
<packageinstances>
<packageinstance name="B06B-PASK"/>
</packageinstances>
</package3d>
<package3d name="B05B-PASK" urn="urn:adsk.eagle:package:11903790/5" type="model" library_version="3">
<packageinstances>
<packageinstance name="B05B-PASK"/>
</packageinstances>
</package3d>
<package3d name="DIP787W45P254L3556H508Q28" urn="urn:adsk.eagle:package:16498237/2" type="model" library_version="3">
<packageinstances>
<packageinstance name="DIP787W45P254L3556H508Q28"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="JST6" urn="urn:adsk.eagle:symbol:11903783/1" library_version="1" library_locally_modified="yes">
<pin name="1" x="12.7" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="2" x="7.62" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="3" x="2.54" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="4" x="-2.54" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="5" x="-7.62" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="6" x="-12.7" y="7.62" visible="pin" length="middle" rot="R270"/>
<wire x1="-15.24" y1="2.54" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="2.54" x2="15.24" y2="-2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-2.54" x2="-15.24" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-15.24" y1="2.54" x2="-15.24" y2="-2.54" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
<symbol name="JST5" urn="urn:adsk.eagle:symbol:11903785/1" library_version="1" library_locally_modified="yes">
<pin name="1" x="10.16" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="2" x="5.08" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="3" x="0" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="4" x="-5.08" y="7.62" visible="pin" length="middle" rot="R270"/>
<pin name="5" x="-10.16" y="7.62" visible="pin" length="middle" rot="R270"/>
<wire x1="-12.7" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="2.54" x2="12.7" y2="-2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-2.54" x2="-12.7" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-12.7" y1="2.54" x2="-12.7" y2="-2.54" width="0.254" layer="94"/>
<text x="-5.08" y="-5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
<symbol name="MCP23017-E/SP" urn="urn:adsk.eagle:symbol:16498236/1" library_version="1">
<wire x1="10.16" y1="27.94" x2="10.16" y2="-27.94" width="0.254" layer="94"/>
<wire x1="10.16" y1="-27.94" x2="-10.16" y2="-27.94" width="0.254" layer="94"/>
<wire x1="-10.16" y1="-27.94" x2="-10.16" y2="27.94" width="0.254" layer="94"/>
<wire x1="-10.16" y1="27.94" x2="10.16" y2="27.94" width="0.254" layer="94"/>
<text x="-10.1794" y="29.0065" size="1.7814" layer="95">&gt;NAME</text>
<text x="-9.148490625" y="-30.7479" size="1.77886875" layer="96">&gt;VALUE</text>
<pin name="VDD" x="15.24" y="25.4" length="middle" direction="pwr" rot="R180"/>
<pin name="VSS" x="15.24" y="-25.4" length="middle" direction="pwr" rot="R180"/>
<pin name="SCL" x="-15.24" y="15.24" length="middle" direction="in" function="clk"/>
<pin name="SDA" x="-15.24" y="12.7" length="middle"/>
<pin name="A0" x="-15.24" y="7.62" length="middle" direction="in"/>
<pin name="A1" x="-15.24" y="5.08" length="middle" direction="in"/>
<pin name="A2" x="-15.24" y="2.54" length="middle" direction="in"/>
<pin name="!RESET" x="-15.24" y="20.32" length="middle" direction="in"/>
<pin name="INTA" x="15.24" y="15.24" length="middle" direction="out" rot="R180"/>
<pin name="INTB" x="15.24" y="12.7" length="middle" direction="out" rot="R180"/>
<pin name="GPA0" x="-15.24" y="-2.54" length="middle"/>
<pin name="GPA1" x="-15.24" y="-5.08" length="middle"/>
<pin name="GPA2" x="-15.24" y="-7.62" length="middle"/>
<pin name="GPA3" x="-15.24" y="-10.16" length="middle"/>
<pin name="GPA4" x="-15.24" y="-12.7" length="middle"/>
<pin name="GPA5" x="-15.24" y="-15.24" length="middle"/>
<pin name="GPA6" x="-15.24" y="-17.78" length="middle"/>
<pin name="GPA7" x="-15.24" y="-20.32" length="middle"/>
<pin name="GPB0" x="15.24" y="-2.54" length="middle" rot="R180"/>
<pin name="GPB1" x="15.24" y="-5.08" length="middle" rot="R180"/>
<pin name="GPB2" x="15.24" y="-7.62" length="middle" rot="R180"/>
<pin name="GPB3" x="15.24" y="-10.16" length="middle" rot="R180"/>
<pin name="GPB4" x="15.24" y="-12.7" length="middle" rot="R180"/>
<pin name="GPB5" x="15.24" y="-15.24" length="middle" rot="R180"/>
<pin name="GPB6" x="15.24" y="-17.78" length="middle" rot="R180"/>
<pin name="GPB7" x="15.24" y="-20.32" length="middle" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="JST6" urn="urn:adsk.eagle:component:11903793/3" prefix="J" library_version="3">
<gates>
<gate name="G$1" symbol="JST6" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B06B-PASK">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
<connect gate="G$1" pin="3" pad="P$3"/>
<connect gate="G$1" pin="4" pad="P$4"/>
<connect gate="G$1" pin="5" pad="P$5"/>
<connect gate="G$1" pin="6" pad="P$6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:11903788/5"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="JST5" urn="urn:adsk.eagle:component:11903795/3" prefix="J" library_version="3">
<gates>
<gate name="G$1" symbol="JST5" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B05B-PASK">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
<connect gate="G$1" pin="3" pad="P$3"/>
<connect gate="G$1" pin="4" pad="P$4"/>
<connect gate="G$1" pin="5" pad="P$5"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:11903790/5"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MCP23017-E/SP" urn="urn:adsk.eagle:component:16498238/2" prefix="U" library_version="3">
<description>MCP23017 Series 5.5 V 1.7 MHz SMT 16-Bit I2C I/O Expander - SPDIP-28</description>
<gates>
<gate name="G$1" symbol="MCP23017-E/SP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DIP787W45P254L3556H508Q28">
<connects>
<connect gate="G$1" pin="!RESET" pad="18"/>
<connect gate="G$1" pin="A0" pad="15"/>
<connect gate="G$1" pin="A1" pad="16"/>
<connect gate="G$1" pin="A2" pad="17"/>
<connect gate="G$1" pin="GPA0" pad="21"/>
<connect gate="G$1" pin="GPA1" pad="22"/>
<connect gate="G$1" pin="GPA2" pad="23"/>
<connect gate="G$1" pin="GPA3" pad="24"/>
<connect gate="G$1" pin="GPA4" pad="25"/>
<connect gate="G$1" pin="GPA5" pad="26"/>
<connect gate="G$1" pin="GPA6" pad="27"/>
<connect gate="G$1" pin="GPA7" pad="28"/>
<connect gate="G$1" pin="GPB0" pad="1"/>
<connect gate="G$1" pin="GPB1" pad="2"/>
<connect gate="G$1" pin="GPB2" pad="3"/>
<connect gate="G$1" pin="GPB3" pad="4"/>
<connect gate="G$1" pin="GPB4" pad="5"/>
<connect gate="G$1" pin="GPB5" pad="6"/>
<connect gate="G$1" pin="GPB6" pad="7"/>
<connect gate="G$1" pin="GPB7" pad="8"/>
<connect gate="G$1" pin="INTA" pad="20"/>
<connect gate="G$1" pin="INTB" pad="19"/>
<connect gate="G$1" pin="SCL" pad="12"/>
<connect gate="G$1" pin="SDA" pad="13"/>
<connect gate="G$1" pin="VDD" pad="9"/>
<connect gate="G$1" pin="VSS" pad="10"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16498237/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="AVAILABILITY" value="Unavailable"/>
<attribute name="DESCRIPTION" value=" I/O Expander 16 I²C 1.7MHz 28-SPDIP "/>
<attribute name="MF" value="Microchip"/>
<attribute name="MP" value="MCP23017-E/SP"/>
<attribute name="PACKAGE" value="DIP-28 Microchip"/>
<attribute name="PRICE" value="None"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply2" urn="urn:adsk.eagle:library:372">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
Please keep in mind, that these devices are necessary for the
automatic wiring of the supply signals.&lt;p&gt;
The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26990/1" library_version="2">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="VCC" urn="urn:adsk.eagle:symbol:26997/1" library_version="2">
<circle x="0" y="1.27" radius="1.27" width="0.254" layer="94"/>
<text x="-1.905" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:27037/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" urn="urn:adsk.eagle:component:27060/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="rcl" urn="urn:adsk.eagle:library:334">
<description>&lt;b&gt;Resistors, Capacitors, Inductors&lt;/b&gt;&lt;p&gt;
Based on the previous libraries:
&lt;ul&gt;
&lt;li&gt;r.lbr
&lt;li&gt;cap.lbr 
&lt;li&gt;cap-fe.lbr
&lt;li&gt;captant.lbr
&lt;li&gt;polcap.lbr
&lt;li&gt;ipc-smd.lbr
&lt;/ul&gt;
All SMD packages are defined according to the IPC specifications and  CECC&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
for Electrolyt Capacitors see also :&lt;p&gt;
www.bccomponents.com &lt;p&gt;
www.panasonic.com&lt;p&gt;
www.kemet.com&lt;p&gt;
http://www.secc.co.jp/pdf/os_e/2004/e_os_all.pdf &lt;b&gt;(SANYO)&lt;/b&gt;
&lt;p&gt;
for trimmer refence see : &lt;u&gt;www.electrospec-inc.com/cross_references/trimpotcrossref.asp&lt;/u&gt;&lt;p&gt;

&lt;table border=0 cellspacing=0 cellpadding=0 width="100%" cellpaddding=0&gt;
&lt;tr valign="top"&gt;

&lt;! &lt;td width="10"&gt;&amp;nbsp;&lt;/td&gt;
&lt;td width="90%"&gt;

&lt;b&gt;&lt;font color="#0000FF" size="4"&gt;TRIM-POT CROSS REFERENCE&lt;/font&gt;&lt;/b&gt;
&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=2&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;RECTANGULAR MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BOURNS&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BI&amp;nbsp;TECH&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;DALE-VISHAY&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PHILIPS/MEPCO&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MURATA&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PANASONIC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;SPECTROL&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MILSPEC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;&lt;TD&gt;&amp;nbsp;&lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3 &gt;
      3005P&lt;BR&gt;
      3006P&lt;BR&gt;
      3006W&lt;BR&gt;
      3006Y&lt;BR&gt;
      3009P&lt;BR&gt;
      3009W&lt;BR&gt;
      3009Y&lt;BR&gt;
      3057J&lt;BR&gt;
      3057L&lt;BR&gt;
      3057P&lt;BR&gt;
      3057Y&lt;BR&gt;
      3059J&lt;BR&gt;
      3059L&lt;BR&gt;
      3059P&lt;BR&gt;
      3059Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      89P&lt;BR&gt;
      89W&lt;BR&gt;
      89X&lt;BR&gt;
      89PH&lt;BR&gt;
      76P&lt;BR&gt;
      89XH&lt;BR&gt;
      78SLT&lt;BR&gt;
      78L&amp;nbsp;ALT&lt;BR&gt;
      56P&amp;nbsp;ALT&lt;BR&gt;
      78P&amp;nbsp;ALT&lt;BR&gt;
      T8S&lt;BR&gt;
      78L&lt;BR&gt;
      56P&lt;BR&gt;
      78P&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      T18/784&lt;BR&gt;
      783&lt;BR&gt;
      781&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2199&lt;BR&gt;
      1697/1897&lt;BR&gt;
      1680/1880&lt;BR&gt;
      2187&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      8035EKP/CT20/RJ-20P&lt;BR&gt;
      -&lt;BR&gt;
      RJ-20X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      1211L&lt;BR&gt;
      8012EKQ&amp;nbsp;ALT&lt;BR&gt;
      8012EKR&amp;nbsp;ALT&lt;BR&gt;
      1211P&lt;BR&gt;
      8012EKJ&lt;BR&gt;
      8012EKL&lt;BR&gt;
      8012EKQ&lt;BR&gt;
      8012EKR&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      2101P&lt;BR&gt;
      2101W&lt;BR&gt;
      2101Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2102L&lt;BR&gt;
      2102S&lt;BR&gt;
      2102Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVMCOG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      43P&lt;BR&gt;
      43W&lt;BR&gt;
      43Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      40L&lt;BR&gt;
      40P&lt;BR&gt;
      40Y&lt;BR&gt;
      70Y-T602&lt;BR&gt;
      70L&lt;BR&gt;
      70P&lt;BR&gt;
      70Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SQUARE MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
   &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3250L&lt;BR&gt;
      3250P&lt;BR&gt;
      3250W&lt;BR&gt;
      3250X&lt;BR&gt;
      3252P&lt;BR&gt;
      3252W&lt;BR&gt;
      3252X&lt;BR&gt;
      3260P&lt;BR&gt;
      3260W&lt;BR&gt;
      3260X&lt;BR&gt;
      3262P&lt;BR&gt;
      3262W&lt;BR&gt;
      3262X&lt;BR&gt;
      3266P&lt;BR&gt;
      3266W&lt;BR&gt;
      3266X&lt;BR&gt;
      3290H&lt;BR&gt;
      3290P&lt;BR&gt;
      3290W&lt;BR&gt;
      3292P&lt;BR&gt;
      3292W&lt;BR&gt;
      3292X&lt;BR&gt;
      3296P&lt;BR&gt;
      3296W&lt;BR&gt;
      3296X&lt;BR&gt;
      3296Y&lt;BR&gt;
      3296Z&lt;BR&gt;
      3299P&lt;BR&gt;
      3299W&lt;BR&gt;
      3299X&lt;BR&gt;
      3299Y&lt;BR&gt;
      3299Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64P&amp;nbsp;ALT&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      64X&amp;nbsp;ALT&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66P&lt;BR&gt;
      66W&lt;BR&gt;
      66X&lt;BR&gt;
      67P&lt;BR&gt;
      67W&lt;BR&gt;
      67X&lt;BR&gt;
      67Y&lt;BR&gt;
      67Z&lt;BR&gt;
      68P&lt;BR&gt;
      68W&lt;BR&gt;
      68X&lt;BR&gt;
      67Y&amp;nbsp;ALT&lt;BR&gt;
      67Z&amp;nbsp;ALT&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      5050&lt;BR&gt;
      5091&lt;BR&gt;
      5080&lt;BR&gt;
      5087&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T63YB&lt;BR&gt;
      T63XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      5887&lt;BR&gt;
      5891&lt;BR&gt;
      5880&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T93Z&lt;BR&gt;
      T93YA&lt;BR&gt;
      T93XA&lt;BR&gt;
      T93YB&lt;BR&gt;
      T93XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKW&lt;BR&gt;
      8026EKM&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKB&lt;BR&gt;
      8026EKM&lt;BR&gt;
      1309X&lt;BR&gt;
      1309P&lt;BR&gt;
      1309W&lt;BR&gt;
      8024EKP&lt;BR&gt;
      8024EKW&lt;BR&gt;
      8024EKN&lt;BR&gt;
      RJ-9P/CT9P&lt;BR&gt;
      RJ-9W&lt;BR&gt;
      RJ-9X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3105P/3106P&lt;BR&gt;
      3105W/3106W&lt;BR&gt;
      3105X/3106X&lt;BR&gt;
      3105Y/3106Y&lt;BR&gt;
      3105Z/3105Z&lt;BR&gt;
      3102P&lt;BR&gt;
      3102W&lt;BR&gt;
      3102X&lt;BR&gt;
      3102Y&lt;BR&gt;
      3102Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMCBG&lt;BR&gt;
      EVMCCG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      55-1-X&lt;BR&gt;
      55-4-X&lt;BR&gt;
      55-3-X&lt;BR&gt;
      55-2-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      50-2-X&lt;BR&gt;
      50-4-X&lt;BR&gt;
      50-3-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      64Y&lt;BR&gt;
      64Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3323P&lt;BR&gt;
      3323S&lt;BR&gt;
      3323W&lt;BR&gt;
      3329H&lt;BR&gt;
      3329P&lt;BR&gt;
      3329W&lt;BR&gt;
      3339H&lt;BR&gt;
      3339P&lt;BR&gt;
      3339W&lt;BR&gt;
      3352E&lt;BR&gt;
      3352H&lt;BR&gt;
      3352K&lt;BR&gt;
      3352P&lt;BR&gt;
      3352T&lt;BR&gt;
      3352V&lt;BR&gt;
      3352W&lt;BR&gt;
      3362H&lt;BR&gt;
      3362M&lt;BR&gt;
      3362P&lt;BR&gt;
      3362R&lt;BR&gt;
      3362S&lt;BR&gt;
      3362U&lt;BR&gt;
      3362W&lt;BR&gt;
      3362X&lt;BR&gt;
      3386B&lt;BR&gt;
      3386C&lt;BR&gt;
      3386F&lt;BR&gt;
      3386H&lt;BR&gt;
      3386K&lt;BR&gt;
      3386M&lt;BR&gt;
      3386P&lt;BR&gt;
      3386S&lt;BR&gt;
      3386W&lt;BR&gt;
      3386X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      25P&lt;BR&gt;
      25S&lt;BR&gt;
      25RX&lt;BR&gt;
      82P&lt;BR&gt;
      82M&lt;BR&gt;
      82PA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      91E&lt;BR&gt;
      91X&lt;BR&gt;
      91T&lt;BR&gt;
      91B&lt;BR&gt;
      91A&lt;BR&gt;
      91V&lt;BR&gt;
      91W&lt;BR&gt;
      25W&lt;BR&gt;
      25V&lt;BR&gt;
      25P&lt;BR&gt;
      -&lt;BR&gt;
      25S&lt;BR&gt;
      25U&lt;BR&gt;
      25RX&lt;BR&gt;
      25X&lt;BR&gt;
      72XW&lt;BR&gt;
      72XL&lt;BR&gt;
      72PM&lt;BR&gt;
      72RX&lt;BR&gt;
      -&lt;BR&gt;
      72PX&lt;BR&gt;
      72P&lt;BR&gt;
      72RXW&lt;BR&gt;
      72RXL&lt;BR&gt;
      72X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T7YB&lt;BR&gt;
      T7YA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      TXD&lt;BR&gt;
      TYA&lt;BR&gt;
      TYP&lt;BR&gt;
      -&lt;BR&gt;
      TYD&lt;BR&gt;
      TX&lt;BR&gt;
      -&lt;BR&gt;
      150SX&lt;BR&gt;
      100SX&lt;BR&gt;
      102T&lt;BR&gt;
      101S&lt;BR&gt;
      190T&lt;BR&gt;
      150TX&lt;BR&gt;
      101&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      101SX&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ET6P&lt;BR&gt;
      ET6S&lt;BR&gt;
      ET6X&lt;BR&gt;
      RJ-6W/8014EMW&lt;BR&gt;
      RJ-6P/8014EMP&lt;BR&gt;
      RJ-6X/8014EMX&lt;BR&gt;
      TM7W&lt;BR&gt;
      TM7P&lt;BR&gt;
      TM7X&lt;BR&gt;
      -&lt;BR&gt;
      8017SMS&lt;BR&gt;
      -&lt;BR&gt;
      8017SMB&lt;BR&gt;
      8017SMA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      CT-6W&lt;BR&gt;
      CT-6H&lt;BR&gt;
      CT-6P&lt;BR&gt;
      CT-6R&lt;BR&gt;
      -&lt;BR&gt;
      CT-6V&lt;BR&gt;
      CT-6X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKV&lt;BR&gt;
      -&lt;BR&gt;
      8038EKX&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKP&lt;BR&gt;
      8038EKZ&lt;BR&gt;
      8038EKW&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3321H&lt;BR&gt;
      3321P&lt;BR&gt;
      3321N&lt;BR&gt;
      1102H&lt;BR&gt;
      1102P&lt;BR&gt;
      1102T&lt;BR&gt;
      RVA0911V304A&lt;BR&gt;
      -&lt;BR&gt;
      RVA0911H413A&lt;BR&gt;
      RVG0707V100A&lt;BR&gt;
      RVA0607V(H)306A&lt;BR&gt;
      RVA1214H213A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3104B&lt;BR&gt;
      3104C&lt;BR&gt;
      3104F&lt;BR&gt;
      3104H&lt;BR&gt;
      -&lt;BR&gt;
      3104M&lt;BR&gt;
      3104P&lt;BR&gt;
      3104S&lt;BR&gt;
      3104W&lt;BR&gt;
      3104X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      EVMQ0G&lt;BR&gt;
      EVMQIG&lt;BR&gt;
      EVMQ3G&lt;BR&gt;
      EVMS0G&lt;BR&gt;
      EVMQ0G&lt;BR&gt;
      EVMG0G&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMK4GA00B&lt;BR&gt;
      EVM30GA00B&lt;BR&gt;
      EVMK0GA00B&lt;BR&gt;
      EVM38GA00B&lt;BR&gt;
      EVMB6&lt;BR&gt;
      EVLQ0&lt;BR&gt;
      -&lt;BR&gt;
      EVMMSG&lt;BR&gt;
      EVMMBG&lt;BR&gt;
      EVMMAG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMMCS&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM0&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM3&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      62-3-1&lt;BR&gt;
      62-1-2&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67R&lt;BR&gt;
      -&lt;BR&gt;
      67P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67X&lt;BR&gt;
      63V&lt;BR&gt;
      63S&lt;BR&gt;
      63M&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63H&lt;BR&gt;
      63P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;&amp;nbsp;&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=3&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT color="#0000FF" SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SMD TRIM-POT CROSS REFERENCE&lt;/B&gt;&lt;/FONT&gt;
      &lt;P&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3224G&lt;BR&gt;
      3224J&lt;BR&gt;
      3224W&lt;BR&gt;
      3269P&lt;BR&gt;
      3269W&lt;BR&gt;
      3269X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      44G&lt;BR&gt;
      44J&lt;BR&gt;
      44W&lt;BR&gt;
      84P&lt;BR&gt;
      84W&lt;BR&gt;
      84X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST63Z&lt;BR&gt;
      ST63Y&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST5P&lt;BR&gt;
      ST5W&lt;BR&gt;
      ST5X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3314G&lt;BR&gt;
      3314J&lt;BR&gt;
      3364A/B&lt;BR&gt;
      3364C/D&lt;BR&gt;
      3364W/X&lt;BR&gt;
      3313G&lt;BR&gt;
      3313J&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      23B&lt;BR&gt;
      23A&lt;BR&gt;
      21X&lt;BR&gt;
      21W&lt;BR&gt;
      -&lt;BR&gt;
      22B&lt;BR&gt;
      22A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST5YL/ST53YL&lt;BR&gt;
      ST5YJ/5T53YJ&lt;BR&gt;
      ST-23A&lt;BR&gt;
      ST-22B&lt;BR&gt;
      ST-22&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST-4B&lt;BR&gt;
      ST-4A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST-3B&lt;BR&gt;
      ST-3A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVM-6YS&lt;BR&gt;
      EVM-1E&lt;BR&gt;
      EVM-1G&lt;BR&gt;
      EVM-1D&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      G4B&lt;BR&gt;
      G4A&lt;BR&gt;
      TR04-3S1&lt;BR&gt;
      TRG04-2S1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      DVR-43A&lt;BR&gt;
      CVR-42C&lt;BR&gt;
      CVR-42A/C&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;
&lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;ALT =&amp;nbsp;ALTERNATE&lt;/B&gt;&lt;/FONT&gt;
&lt;P&gt;

&amp;nbsp;
&lt;P&gt;
&lt;/td&gt;
&lt;/tr&gt;
&lt;/table&gt;</description>
<packages>
<package name="R0402" urn="urn:adsk.eagle:footprint:23043/3" library_version="5">
<description>&lt;b&gt;Chip RESISTOR 0402 EIA (1005 Metric)&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1" y1="0.483" x2="1" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1" y1="0.483" x2="1" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1" y1="-0.483" x2="-1" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1" y1="-0.483" x2="-1" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.5" y="0" dx="0.6" dy="0.7" layer="1"/>
<smd name="2" x="0.5" y="0" dx="0.6" dy="0.7" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.35" x2="0.1999" y2="0.35" layer="35"/>
</package>
<package name="R0603" urn="urn:adsk.eagle:footprint:23044/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0805" urn="urn:adsk.eagle:footprint:23045/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R0805W" urn="urn:adsk.eagle:footprint:23046/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R1206" urn="urn:adsk.eagle:footprint:23047/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1206W" urn="urn:adsk.eagle:footprint:23048/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1210" urn="urn:adsk.eagle:footprint:23049/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
</package>
<package name="R1210W" urn="urn:adsk.eagle:footprint:23050/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="R2010" urn="urn:adsk.eagle:footprint:23051/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2010W" urn="urn:adsk.eagle:footprint:23052/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2012" urn="urn:adsk.eagle:footprint:23053/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2012W" urn="urn:adsk.eagle:footprint:23054/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2512" urn="urn:adsk.eagle:footprint:23055/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R2512W" urn="urn:adsk.eagle:footprint:23056/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R3216" urn="urn:adsk.eagle:footprint:23057/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3216W" urn="urn:adsk.eagle:footprint:23058/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3225" urn="urn:adsk.eagle:footprint:23059/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R3225W" urn="urn:adsk.eagle:footprint:23060/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R5025" urn="urn:adsk.eagle:footprint:23061/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R5025W" urn="urn:adsk.eagle:footprint:23062/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332" urn="urn:adsk.eagle:footprint:23063/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.1" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="2" x="3.1" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332W" urn="urn:adsk.eagle:footprint:25646/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<smd name="2" x="3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M0805" urn="urn:adsk.eagle:footprint:23065/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M1206" urn="urn:adsk.eagle:footprint:23066/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M1406" urn="urn:adsk.eagle:footprint:23067/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="M2012" urn="urn:adsk.eagle:footprint:23068/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M2309" urn="urn:adsk.eagle:footprint:23069/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M3216" urn="urn:adsk.eagle:footprint:23070/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M3516" urn="urn:adsk.eagle:footprint:23071/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="M5923" urn="urn:adsk.eagle:footprint:23072/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="0204/5" urn="urn:adsk.eagle:footprint:22991/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0" x2="-2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.778" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.635" x2="1.778" y2="0.635" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.0066" y="1.1684" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-0.254" x2="-1.778" y2="0.254" layer="51"/>
<rectangle x1="1.778" y1="-0.254" x2="2.032" y2="0.254" layer="51"/>
</package>
<package name="0204/7" urn="urn:adsk.eagle:footprint:22998/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="0204V" urn="urn:adsk.eagle:footprint:22999/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.508" layer="51"/>
<wire x1="-0.127" y1="0" x2="0.127" y2="0" width="0.508" layer="21"/>
<circle x="-1.27" y="0" radius="0.889" width="0.1524" layer="51"/>
<circle x="-1.27" y="0" radius="0.635" width="0.0508" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.1336" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/10" urn="urn:adsk.eagle:footprint:22992/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
</package>
<package name="0207/12" urn="urn:adsk.eagle:footprint:22993/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 12 mm</description>
<wire x1="6.35" y1="0" x2="5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="4.445" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-4.445" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.3086" y2="0.3048" layer="21"/>
<rectangle x1="-5.3086" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
</package>
<package name="0207/15" urn="urn:adsk.eagle:footprint:22997/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 15mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="5.715" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-5.715" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="5.715" y1="-0.3048" x2="6.5786" y2="0.3048" layer="21"/>
<rectangle x1="-6.5786" y1="-0.3048" x2="-5.715" y2="0.3048" layer="21"/>
</package>
<package name="0207/2V" urn="urn:adsk.eagle:footprint:22994/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="-0.381" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.6096" layer="21"/>
<wire x1="0.381" y1="0" x2="1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.27" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-0.0508" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0508" y="-2.2352" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/5V" urn="urn:adsk.eagle:footprint:22995/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-0.889" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.762" y1="0" x2="0.762" y2="0" width="0.6096" layer="21"/>
<wire x1="0.889" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<circle x="-2.54" y="0" radius="1.27" width="0.1016" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.143" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.143" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/7" urn="urn:adsk.eagle:footprint:22996/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 7.5 mm</description>
<wire x1="-3.81" y1="0" x2="-3.429" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0" x2="3.81" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.5588" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-3.429" y1="-0.3048" x2="-3.175" y2="0.3048" layer="51"/>
<rectangle x1="3.175" y1="-0.3048" x2="3.429" y2="0.3048" layer="51"/>
</package>
<package name="0309/10" urn="urn:adsk.eagle:footprint:23073/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 10mm</description>
<wire x1="-4.699" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="5.08" y1="0" x2="4.699" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.6228" y1="-0.3048" x2="-4.318" y2="0.3048" layer="51"/>
<rectangle x1="4.318" y1="-0.3048" x2="4.6228" y2="0.3048" layer="51"/>
</package>
<package name="0309/12" urn="urn:adsk.eagle:footprint:23074/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.318" y1="-0.3048" x2="5.1816" y2="0.3048" layer="21"/>
<rectangle x1="-5.1816" y1="-0.3048" x2="-4.318" y2="0.3048" layer="21"/>
</package>
<package name="0309V" urn="urn:adsk.eagle:footprint:23075/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 2.5 mm</description>
<wire x1="1.27" y1="0" x2="0.635" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.635" y1="0" x2="-1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.524" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="0.254" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.254" y="-2.2098" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="0.254" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.635" y1="-0.3048" x2="-0.3302" y2="0.3048" layer="51"/>
<rectangle x1="-0.3302" y1="-0.3048" x2="0.254" y2="0.3048" layer="21"/>
</package>
<package name="0411/12" urn="urn:adsk.eagle:footprint:23076/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.762" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.762" layer="51"/>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.3594" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
<rectangle x1="5.08" y1="-0.381" x2="5.3594" y2="0.381" layer="21"/>
</package>
<package name="0411/15" urn="urn:adsk.eagle:footprint:23077/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 15 mm</description>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-6.35" y2="0" width="0.762" layer="51"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.762" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="5.08" y1="-0.381" x2="6.477" y2="0.381" layer="21"/>
<rectangle x1="-6.477" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
</package>
<package name="0411V" urn="urn:adsk.eagle:footprint:23078/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 3.81 mm</description>
<wire x1="1.27" y1="0" x2="0.3048" y2="0" width="0.762" layer="51"/>
<wire x1="-1.5748" y1="0" x2="-2.54" y2="0" width="0.762" layer="51"/>
<circle x="-2.54" y="0" radius="2.032" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.9144" shape="octagon"/>
<text x="-0.508" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.5334" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.4732" y1="-0.381" x2="0.2032" y2="0.381" layer="21"/>
</package>
<package name="0414/15" urn="urn:adsk.eagle:footprint:23079/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="1.905" x2="-5.842" y2="2.159" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-5.842" y2="-2.159" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="-2.159" x2="6.096" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="2.159" x2="6.096" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-6.096" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="2.159" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="2.032" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="-2.159" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="-4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.842" y1="2.159" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.842" y1="-2.159" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.905" x2="6.096" y2="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.5654" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="6.096" y1="-0.4064" x2="6.5024" y2="0.4064" layer="21"/>
<rectangle x1="-6.5024" y1="-0.4064" x2="-6.096" y2="0.4064" layer="21"/>
</package>
<package name="0414V" urn="urn:adsk.eagle:footprint:23080/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.159" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.381" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.381" y="-2.3622" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.2954" y2="0.4064" layer="21"/>
</package>
<package name="0617/17" urn="urn:adsk.eagle:footprint:23081/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 17.5 mm</description>
<wire x1="-8.89" y1="0" x2="-8.636" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.636" y1="0" x2="8.89" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.016" shape="octagon"/>
<text x="-8.128" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.096" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-8.5344" y1="-0.4064" x2="-8.2296" y2="0.4064" layer="51"/>
<rectangle x1="8.2296" y1="-0.4064" x2="8.5344" y2="0.4064" layer="51"/>
</package>
<package name="0617/22" urn="urn:adsk.eagle:footprint:23082/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 22.5 mm</description>
<wire x1="-10.287" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.287" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.255" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.1854" y1="-0.4064" x2="-8.255" y2="0.4064" layer="21"/>
<rectangle x1="8.255" y1="-0.4064" x2="10.1854" y2="0.4064" layer="21"/>
</package>
<package name="0617V" urn="urn:adsk.eagle:footprint:23083/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="3.048" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="0.635" y="1.4224" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.635" y="-2.6162" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.3208" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="0922/22" urn="urn:adsk.eagle:footprint:23084/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 22.5 mm</description>
<wire x1="11.43" y1="0" x2="10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-11.43" y1="0" x2="-10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-10.16" y1="-4.191" x2="-10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="4.572" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="4.318" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="-4.572" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="-4.318" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="-8.636" y2="4.318" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="-8.636" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="9.779" y1="4.572" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="9.779" y1="-4.572" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-4.191" x2="10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-4.191" x2="-9.779" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-10.16" y1="4.191" x2="-9.779" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="9.779" y1="-4.572" x2="10.16" y2="-4.191" width="0.1524" layer="21" curve="90"/>
<wire x1="9.779" y1="4.572" x2="10.16" y2="4.191" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-10.16" y="5.1054" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.7188" y1="-0.4064" x2="-10.16" y2="0.4064" layer="51"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-10.16" y2="0.4064" layer="21"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.7188" y2="0.4064" layer="51"/>
</package>
<package name="P0613V" urn="urn:adsk.eagle:footprint:23085/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.254" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.254" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="P0613/15" urn="urn:adsk.eagle:footprint:23086/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.032" x2="-6.223" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.032" x2="-6.223" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="-2.286" x2="6.477" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="2.286" x2="6.477" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.223" y1="2.286" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.159" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-6.223" y1="-2.286" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.159" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="-5.207" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="-5.207" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.223" y1="2.286" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-2.286" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="0.635" width="0.1524" layer="51"/>
<wire x1="6.477" y1="2.032" x2="6.477" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.032" x2="-6.477" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="2.032" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.477" y="2.6924" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-7.0358" y1="-0.4064" x2="-6.477" y2="0.4064" layer="51"/>
<rectangle x1="6.477" y1="-0.4064" x2="7.0358" y2="0.4064" layer="51"/>
</package>
<package name="P0817/22" urn="urn:adsk.eagle:footprint:23087/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 22.5 mm</description>
<wire x1="-10.414" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="-3.429" x2="-8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="3.81" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="3.556" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="-3.81" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-3.556" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="-6.985" y2="3.556" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="-6.985" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.128" y1="3.81" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="8.128" y1="-3.81" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.429" x2="8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.414" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="3.429" x2="-8.128" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.509" y1="-3.429" x2="-8.128" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="8.128" y1="3.81" x2="8.509" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.128" y1="-3.81" x2="8.509" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="4.2164" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.604" y="-2.2606" size="1.27" layer="51" ratio="10" rot="R90">0817</text>
<rectangle x1="8.509" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-8.509" y2="0.4064" layer="21"/>
</package>
<package name="P0817V" urn="urn:adsk.eagle:footprint:23088/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 6.35 mm</description>
<wire x1="-3.81" y1="0" x2="-5.08" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="3.81" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="octagon"/>
<text x="-1.016" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.032" size="1.016" layer="21" ratio="12">0817</text>
<rectangle x1="-3.81" y1="-0.4064" x2="0" y2="0.4064" layer="21"/>
</package>
<package name="V234/12" urn="urn:adsk.eagle:footprint:23089/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V234, grid 12.5 mm</description>
<wire x1="-4.953" y1="1.524" x2="-4.699" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.953" y1="-1.524" x2="-4.699" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="1.524" x2="-4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="-4.699" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.8128" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.016" shape="octagon"/>
<text x="-4.953" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.953" y1="-0.4064" x2="5.4102" y2="0.4064" layer="21"/>
<rectangle x1="-5.4102" y1="-0.4064" x2="-4.953" y2="0.4064" layer="21"/>
</package>
<package name="V235/17" urn="urn:adsk.eagle:footprint:23090/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V235, grid 17.78 mm</description>
<wire x1="-6.731" y1="2.921" x2="6.731" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-7.112" y1="2.54" x2="-7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.921" x2="-6.731" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0" x2="7.874" y2="0" width="1.016" layer="51"/>
<wire x1="-7.874" y1="0" x2="-8.89" y2="0" width="1.016" layer="51"/>
<wire x1="-7.112" y1="-2.54" x2="-6.731" y2="-2.921" width="0.1524" layer="21" curve="90"/>
<wire x1="6.731" y1="2.921" x2="7.112" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="-2.921" x2="7.112" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-7.112" y1="2.54" x2="-6.731" y2="2.921" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-8.89" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.1938" shape="octagon"/>
<text x="-6.858" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.842" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="7.112" y1="-0.508" x2="7.747" y2="0.508" layer="21"/>
<rectangle x1="-7.747" y1="-0.508" x2="-7.112" y2="0.508" layer="21"/>
</package>
<package name="V526-0" urn="urn:adsk.eagle:footprint:23091/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V526-0, grid 2.5 mm</description>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.413" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102R" urn="urn:adsk.eagle:footprint:23092/1" library_version="5">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102W" urn="urn:adsk.eagle:footprint:23093/1" library_version="5">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<smd name="2" x="0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204R" urn="urn:adsk.eagle:footprint:25676/1" library_version="5">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.938" y1="0.6" x2="-0.938" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.938" y1="-0.6" x2="0.938" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204W" urn="urn:adsk.eagle:footprint:25677/1" library_version="5">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.684" y1="0.6" x2="-0.684" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.684" y1="-0.6" x2="0.684" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207R" urn="urn:adsk.eagle:footprint:25678/1" library_version="5">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.2125" y1="1" x2="-1.2125" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.2125" y1="-1" x2="1.2125" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<smd name="2" x="2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207W" urn="urn:adsk.eagle:footprint:25679/1" library_version="5">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.149" y1="1" x2="-1.149" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.149" y1="-1" x2="1.149" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<smd name="2" x="2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<text x="-2.54" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="0922V" urn="urn:adsk.eagle:footprint:23098/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 7.5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.508" y="1.6764" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.508" y="-2.9972" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.54" size="1.016" layer="21" ratio="12">0922</text>
<rectangle x1="-3.81" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="RDH/15" urn="urn:adsk.eagle:footprint:23099/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type RDH, grid 15 mm</description>
<wire x1="-7.62" y1="0" x2="-6.858" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="3.048" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="2.794" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-3.048" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.794" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="-4.953" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="-4.953" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.667" x2="-6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-2.667" x2="6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.667" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="3.048" x2="6.477" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.667" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="6.096" y1="-3.048" x2="6.477" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.35" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="4.572" y="-1.7272" size="1.27" layer="51" ratio="10" rot="R90">RDH</text>
<rectangle x1="-6.7564" y1="-0.4064" x2="-6.4516" y2="0.4064" layer="51"/>
<rectangle x1="6.4516" y1="-0.4064" x2="6.7564" y2="0.4064" layer="51"/>
</package>
<package name="MINI_MELF-0102AX" urn="urn:adsk.eagle:footprint:23100/1" library_version="5">
<description>&lt;b&gt;Mini MELF 0102 Axial&lt;/b&gt;</description>
<circle x="0" y="0" radius="0.6" width="0" layer="51"/>
<circle x="0" y="0" radius="0.6" width="0" layer="52"/>
<smd name="1" x="0" y="0" dx="1.9" dy="1.9" layer="1" roundness="100"/>
<smd name="2" x="0" y="0" dx="1.9" dy="1.9" layer="16" roundness="100"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="1.3"/>
</package>
<package name="R0201" urn="urn:adsk.eagle:footprint:25683/1" library_version="5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; chip&lt;p&gt;
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<smd name="1" x="-0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<smd name="2" x="0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="0.15" layer="21"/>
</package>
<package name="VTA52" urn="urn:adsk.eagle:footprint:25684/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR52&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-15.24" y1="0" x2="-13.97" y2="0" width="0.6096" layer="51"/>
<wire x1="12.6225" y1="0.025" x2="12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="4.725" x2="-12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="4.725" x2="-12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="0.025" x2="-12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="-4.65" x2="12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="-4.65" x2="12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="13.97" y1="0" x2="15.24" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-15.24" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="15.24" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-13.97" y1="-0.3048" x2="-12.5675" y2="0.3048" layer="21"/>
<rectangle x1="12.5675" y1="-0.3048" x2="13.97" y2="0.3048" layer="21"/>
</package>
<package name="VTA53" urn="urn:adsk.eagle:footprint:25685/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR53&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="4.7" x2="-9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="4.7" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-4.675" x2="9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-4.675" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA54" urn="urn:adsk.eagle:footprint:25686/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR54&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="3.3" x2="-9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="3.3" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-3.3" x2="9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-3.3" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA55" urn="urn:adsk.eagle:footprint:25687/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-8.255" y1="0" x2="-6.985" y2="0" width="0.6096" layer="51"/>
<wire x1="6.405" y1="0" x2="6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="3.3" x2="-6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="3.3" x2="-6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="0" x2="-6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="-3.3" x2="6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="-3.3" x2="6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="6.985" y1="0" x2="8.255" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-8.255" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="8.255" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-6.985" y1="-0.3048" x2="-6.35" y2="0.3048" layer="21"/>
<rectangle x1="6.35" y1="-0.3048" x2="6.985" y2="0.3048" layer="21"/>
</package>
<package name="VTA56" urn="urn:adsk.eagle:footprint:25688/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR56&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="4.5" y1="0" x2="4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="3.3" x2="-4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="0" x2="-4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="-3.3" x2="4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="-3.3" x2="4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.08" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.08" y2="0.3048" layer="21"/>
</package>
<package name="VMTA55" urn="urn:adsk.eagle:footprint:25689/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-5.08" y1="0" x2="-4.26" y2="0" width="0.6096" layer="51"/>
<wire x1="3.3375" y1="-1.45" x2="3.3375" y2="1.45" width="0.1524" layer="21"/>
<wire x1="3.3375" y1="1.45" x2="-3.3625" y2="1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="1.45" x2="-3.3625" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="-1.45" x2="3.3375" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="4.235" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.1" shape="octagon"/>
<text x="-3.175" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.26" y1="-0.3048" x2="-3.3075" y2="0.3048" layer="21"/>
<rectangle x1="3.2825" y1="-0.3048" x2="4.235" y2="0.3048" layer="21"/>
</package>
<package name="VMTB60" urn="urn:adsk.eagle:footprint:25690/1" library_version="5">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC60&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.585" y2="0" width="0.6096" layer="51"/>
<wire x1="4.6875" y1="-1.95" x2="4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="4.6875" y1="1.95" x2="-4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="1.95" x2="-4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="-1.95" x2="4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="5.585" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-4.445" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.585" y1="-0.3048" x2="-4.6325" y2="0.3048" layer="21"/>
<rectangle x1="4.6325" y1="-0.3048" x2="5.585" y2="0.3048" layer="21"/>
</package>
<package name="R4527" urn="urn:adsk.eagle:footprint:13246/1" library_version="5">
<description>&lt;b&gt;Package 4527&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<wire x1="-5.675" y1="-3.375" x2="5.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.65" y1="-3.375" x2="5.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.375" x2="-5.675" y2="3.375" width="0.2032" layer="21"/>
<wire x1="-5.675" y1="3.375" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<smd name="1" x="-4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.715" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.715" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0001" urn="urn:adsk.eagle:footprint:25692/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.075" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="1.606" x2="3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-2.544" y="2.229" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.501" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0002" urn="urn:adsk.eagle:footprint:25693/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.55" y1="3.375" x2="-5.55" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.55" y1="-3.375" x2="5.55" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.55" y1="-3.375" x2="5.55" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.55" y1="3.375" x2="-5.55" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.65" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.65" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC01/2" urn="urn:adsk.eagle:footprint:25694/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="-1.475" width="0.2032" layer="51"/>
<wire x1="-2.45" y1="-1.475" x2="2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="1.475" width="0.2032" layer="51"/>
<wire x1="2.45" y1="1.475" x2="-2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="1.106" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="-1.106" x2="-2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="1.106" x2="2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="-1.106" width="0.2032" layer="21"/>
<smd name="1" x="-2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<text x="-2.544" y="1.904" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.176" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC2515" urn="urn:adsk.eagle:footprint:25695/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.05" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.05" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="1.606" x2="3.05" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-3.2" y="2.15" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.2" y="-3.4" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC4527" urn="urn:adsk.eagle:footprint:25696/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.675" y1="3.4" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.675" y1="-3.375" x2="5.675" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.675" y1="-3.375" x2="5.675" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.675" y1="3.4" x2="-5.675" y2="3.4" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.775" y="3.925" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.775" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC6927" urn="urn:adsk.eagle:footprint:25697/1" library_version="5">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-8.65" y1="3.375" x2="-8.65" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-8.65" y1="-3.375" x2="8.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="8.65" y1="-3.375" x2="8.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="8.65" y1="3.375" x2="-8.65" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-7.95" y="0.025" dx="3.94" dy="5.97" layer="1"/>
<smd name="2" x="7.95" y="0" dx="3.94" dy="5.97" layer="1"/>
<text x="-8.75" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-8.75" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1218" urn="urn:adsk.eagle:footprint:25698/1" library_version="5">
<description>&lt;b&gt;CRCW1218 Thick Film, Rectangular Chip Resistors&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com .. dcrcw.pdf</description>
<wire x1="-0.913" y1="-2.219" x2="0.939" y2="-2.219" width="0.1524" layer="51"/>
<wire x1="0.913" y1="2.219" x2="-0.939" y2="2.219" width="0.1524" layer="51"/>
<smd name="1" x="-1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<smd name="2" x="1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<text x="-2.54" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-2.3" x2="-0.9009" y2="2.3" layer="51"/>
<rectangle x1="0.9144" y1="-2.3" x2="1.6645" y2="2.3" layer="51"/>
</package>
<package name="1812X7R" urn="urn:adsk.eagle:footprint:25699/1" library_version="5">
<description>&lt;b&gt;Chip Monolithic Ceramic Capacitors&lt;/b&gt; Medium Voltage High Capacitance for General Use&lt;p&gt;
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<wire x1="-1.1" y1="1.5" x2="1.1" y2="1.5" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.5" x2="-1.1" y2="-1.5" width="0.2032" layer="51"/>
<wire x1="-0.6" y1="1.5" x2="0.6" y2="1.5" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.5" x2="-0.6" y2="-1.5" width="0.2032" layer="21"/>
<smd name="1" x="-1.425" y="0" dx="0.8" dy="3.5" layer="1"/>
<smd name="2" x="1.425" y="0" dx="0.8" dy="3.5" layer="1" rot="R180"/>
<text x="-1.9456" y="1.9958" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.9456" y="-3.7738" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.4" y1="-1.6" x2="-1.1" y2="1.6" layer="51"/>
<rectangle x1="1.1" y1="-1.6" x2="1.4" y2="1.6" layer="51" rot="R180"/>
</package>
<package name="PRL1632" urn="urn:adsk.eagle:footprint:25700/1" library_version="5">
<description>&lt;b&gt;PRL1632 are realized as 1W for 3.2 × 1.6mm(1206)&lt;/b&gt;&lt;p&gt;
Source: http://www.mouser.com/ds/2/392/products_18-2245.pdf</description>
<wire x1="0.7275" y1="-1.5228" x2="-0.7277" y2="-1.5228" width="0.1524" layer="51"/>
<wire x1="0.7275" y1="1.5228" x2="-0.7152" y2="1.5228" width="0.1524" layer="51"/>
<smd name="2" x="0.822" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="1" x="-0.822" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-1.4" y="1.8" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.4" y="-3" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8" y1="-1.6" x2="-0.4" y2="1.6" layer="51"/>
<rectangle x1="0.4" y1="-1.6" x2="0.8" y2="1.6" layer="51"/>
</package>
<package name="R01005" urn="urn:adsk.eagle:footprint:25701/1" library_version="5">
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="R0603" urn="urn:adsk.eagle:package:23555/3" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0603"/>
</packageinstances>
</package3d>
<package3d name="R0805" urn="urn:adsk.eagle:package:23553/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0805"/>
</packageinstances>
</package3d>
<package3d name="R1206" urn="urn:adsk.eagle:package:23540/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1206"/>
</packageinstances>
</package3d>
<package3d name="R1210" urn="urn:adsk.eagle:package:23554/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1210"/>
</packageinstances>
</package3d>
<package3d name="R2010" urn="urn:adsk.eagle:package:23551/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2010"/>
</packageinstances>
</package3d>
<package3d name="R2012" urn="urn:adsk.eagle:package:23543/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2012"/>
</packageinstances>
</package3d>
<package3d name="R2512" urn="urn:adsk.eagle:package:23545/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2512"/>
</packageinstances>
</package3d>
<package3d name="R3216" urn="urn:adsk.eagle:package:23557/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3216"/>
</packageinstances>
</package3d>
<package3d name="R3225" urn="urn:adsk.eagle:package:23549/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3225"/>
</packageinstances>
</package3d>
<package3d name="R5025" urn="urn:adsk.eagle:package:23552/2" type="model" library_version="5">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R5025"/>
</packageinstances>
</package3d>
<package3d name="R6332" urn="urn:adsk.eagle:package:23559/2" type="model" library_version="5">
<description>RESISTOR
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332"/>
</packageinstances>
</package3d>
<package3d name="0204/5" urn="urn:adsk.eagle:package:23488/1" type="box" library_version="5">
<description>RESISTOR
type 0204, grid 5 mm</description>
<packageinstances>
<packageinstance name="0204/5"/>
</packageinstances>
</package3d>
<package3d name="0204V" urn="urn:adsk.eagle:package:23495/1" type="box" library_version="5">
<description>RESISTOR
type 0204, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0204V"/>
</packageinstances>
</package3d>
<package3d name="0207/12" urn="urn:adsk.eagle:package:23489/1" type="box" library_version="5">
<description>RESISTOR
type 0207, grid 12 mm</description>
<packageinstances>
<packageinstance name="0207/12"/>
</packageinstances>
</package3d>
<package3d name="0207/15" urn="urn:adsk.eagle:package:23492/1" type="box" library_version="5">
<description>RESISTOR
type 0207, grid 15mm</description>
<packageinstances>
<packageinstance name="0207/15"/>
</packageinstances>
</package3d>
<package3d name="0207/2V" urn="urn:adsk.eagle:package:23490/1" type="box" library_version="5">
<description>RESISTOR
type 0207, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0207/2V"/>
</packageinstances>
</package3d>
<package3d name="0207/5V" urn="urn:adsk.eagle:package:23502/1" type="box" library_version="5">
<description>RESISTOR
type 0207, grid 5 mm</description>
<packageinstances>
<packageinstance name="0207/5V"/>
</packageinstances>
</package3d>
<package3d name="0309/12" urn="urn:adsk.eagle:package:23571/1" type="box" library_version="5">
<description>RESISTOR
type 0309, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0309/12"/>
</packageinstances>
</package3d>
<package3d name="0309V" urn="urn:adsk.eagle:package:23572/1" type="box" library_version="5">
<description>RESISTOR
type 0309, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0309V"/>
</packageinstances>
</package3d>
<package3d name="0411/12" urn="urn:adsk.eagle:package:23578/1" type="box" library_version="5">
<description>RESISTOR
type 0411, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0411/12"/>
</packageinstances>
</package3d>
<package3d name="0411V" urn="urn:adsk.eagle:package:23570/1" type="box" library_version="5">
<description>RESISTOR
type 0411, grid 3.81 mm</description>
<packageinstances>
<packageinstance name="0411V"/>
</packageinstances>
</package3d>
<package3d name="0414V" urn="urn:adsk.eagle:package:23574/1" type="box" library_version="5">
<description>RESISTOR
type 0414, grid 5 mm</description>
<packageinstances>
<packageinstance name="0414V"/>
</packageinstances>
</package3d>
<package3d name="0617/22" urn="urn:adsk.eagle:package:23577/1" type="box" library_version="5">
<description>RESISTOR
type 0617, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0617/22"/>
</packageinstances>
</package3d>
<package3d name="0617V" urn="urn:adsk.eagle:package:23576/1" type="box" library_version="5">
<description>RESISTOR
type 0617, grid 5 mm</description>
<packageinstances>
<packageinstance name="0617V"/>
</packageinstances>
</package3d>
<package3d name="P0613V" urn="urn:adsk.eagle:package:23582/1" type="box" library_version="5">
<description>RESISTOR
type 0613, grid 5 mm</description>
<packageinstances>
<packageinstance name="P0613V"/>
</packageinstances>
</package3d>
<package3d name="P0817/22" urn="urn:adsk.eagle:package:23583/1" type="box" library_version="5">
<description>RESISTOR
type 0817, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="P0817/22"/>
</packageinstances>
</package3d>
<package3d name="P0817V" urn="urn:adsk.eagle:package:23608/1" type="box" library_version="5">
<description>RESISTOR
type 0817, grid 6.35 mm</description>
<packageinstances>
<packageinstance name="P0817V"/>
</packageinstances>
</package3d>
<package3d name="V234/12" urn="urn:adsk.eagle:package:23592/1" type="box" library_version="5">
<description>RESISTOR
type V234, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="V234/12"/>
</packageinstances>
</package3d>
<package3d name="V526-0" urn="urn:adsk.eagle:package:23590/1" type="box" library_version="5">
<description>RESISTOR
type V526-0, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="V526-0"/>
</packageinstances>
</package3d>
<package3d name="0922V" urn="urn:adsk.eagle:package:23589/1" type="box" library_version="5">
<description>RESISTOR
type 0922, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0922V"/>
</packageinstances>
</package3d>
<package3d name="RDH/15" urn="urn:adsk.eagle:package:23595/1" type="box" library_version="5">
<description>RESISTOR
type RDH, grid 15 mm</description>
<packageinstances>
<packageinstance name="RDH/15"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102AX" urn="urn:adsk.eagle:package:23594/1" type="box" library_version="5">
<description>Mini MELF 0102 Axial</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102AX"/>
</packageinstances>
</package3d>
<package3d name="R0402" urn="urn:adsk.eagle:package:23547/3" type="model" library_version="11">
<description>Chip RESISTOR 0402 EIA (1005 Metric)</description>
<packageinstances>
<packageinstance name="R0402"/>
</packageinstances>
</package3d>
<package3d name="R0805W" urn="urn:adsk.eagle:package:23537/2" type="model" library_version="11">
<description>RESISTOR wave soldering</description>
<packageinstances>
<packageinstance name="R0805W"/>
</packageinstances>
</package3d>
<package3d name="R1206W" urn="urn:adsk.eagle:package:23539/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1206W"/>
</packageinstances>
</package3d>
<package3d name="R1210W" urn="urn:adsk.eagle:package:23541/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1210W"/>
</packageinstances>
</package3d>
<package3d name="R2010W" urn="urn:adsk.eagle:package:23542/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2010W"/>
</packageinstances>
</package3d>
<package3d name="R2012W" urn="urn:adsk.eagle:package:23544/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2012W"/>
</packageinstances>
</package3d>
<package3d name="R2512W" urn="urn:adsk.eagle:package:23565/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2512W"/>
</packageinstances>
</package3d>
<package3d name="R3216W" urn="urn:adsk.eagle:package:23548/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3216W"/>
</packageinstances>
</package3d>
<package3d name="R3225W" urn="urn:adsk.eagle:package:23550/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3225W"/>
</packageinstances>
</package3d>
<package3d name="R5025W" urn="urn:adsk.eagle:package:23558/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R5025W"/>
</packageinstances>
</package3d>
<package3d name="R6332W" urn="urn:adsk.eagle:package:26078/2" type="model" library_version="11">
<description>RESISTOR wave soldering
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332W"/>
</packageinstances>
</package3d>
<package3d name="M0805" urn="urn:adsk.eagle:package:23556/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M0805"/>
</packageinstances>
</package3d>
<package3d name="M1206" urn="urn:adsk.eagle:package:23566/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M1206"/>
</packageinstances>
</package3d>
<package3d name="M1406" urn="urn:adsk.eagle:package:23569/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M1406"/>
</packageinstances>
</package3d>
<package3d name="M2012" urn="urn:adsk.eagle:package:23561/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M2012"/>
</packageinstances>
</package3d>
<package3d name="M2309" urn="urn:adsk.eagle:package:23562/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M2309"/>
</packageinstances>
</package3d>
<package3d name="M3216" urn="urn:adsk.eagle:package:23563/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M3216"/>
</packageinstances>
</package3d>
<package3d name="M3516" urn="urn:adsk.eagle:package:23573/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M3516"/>
</packageinstances>
</package3d>
<package3d name="M5923" urn="urn:adsk.eagle:package:23564/3" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M5923"/>
</packageinstances>
</package3d>
<package3d name="0204/7" urn="urn:adsk.eagle:package:23498/2" type="model" library_version="11">
<description>RESISTOR
type 0204, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0204/7"/>
</packageinstances>
</package3d>
<package3d name="0207/10" urn="urn:adsk.eagle:package:23491/2" type="model" library_version="11">
<description>RESISTOR
type 0207, grid 10 mm</description>
<packageinstances>
<packageinstance name="0207/10"/>
</packageinstances>
</package3d>
<package3d name="0207/7" urn="urn:adsk.eagle:package:23493/2" type="model" library_version="11">
<description>RESISTOR
type 0207, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0207/7"/>
</packageinstances>
</package3d>
<package3d name="0309/10" urn="urn:adsk.eagle:package:23567/2" type="model" library_version="11">
<description>RESISTOR
type 0309, grid 10mm</description>
<packageinstances>
<packageinstance name="0309/10"/>
</packageinstances>
</package3d>
<package3d name="0411/15" urn="urn:adsk.eagle:package:23568/2" type="model" library_version="11">
<description>RESISTOR
type 0411, grid 15 mm</description>
<packageinstances>
<packageinstance name="0411/15"/>
</packageinstances>
</package3d>
<package3d name="0414/15" urn="urn:adsk.eagle:package:23579/2" type="model" library_version="11">
<description>RESISTOR
type 0414, grid 15 mm</description>
<packageinstances>
<packageinstance name="0414/15"/>
</packageinstances>
</package3d>
<package3d name="0617/17" urn="urn:adsk.eagle:package:23575/2" type="model" library_version="11">
<description>RESISTOR
type 0617, grid 17.5 mm</description>
<packageinstances>
<packageinstance name="0617/17"/>
</packageinstances>
</package3d>
<package3d name="0922/22" urn="urn:adsk.eagle:package:23580/2" type="model" library_version="11">
<description>RESISTOR
type 0922, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0922/22"/>
</packageinstances>
</package3d>
<package3d name="P0613/15" urn="urn:adsk.eagle:package:23581/2" type="model" library_version="11">
<description>RESISTOR
type 0613, grid 15 mm</description>
<packageinstances>
<packageinstance name="P0613/15"/>
</packageinstances>
</package3d>
<package3d name="V235/17" urn="urn:adsk.eagle:package:23586/2" type="model" library_version="11">
<description>RESISTOR
type V235, grid 17.78 mm</description>
<packageinstances>
<packageinstance name="V235/17"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102R" urn="urn:adsk.eagle:package:23591/2" type="model" library_version="11">
<description>CECC Size RC2211 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102W" urn="urn:adsk.eagle:package:23588/2" type="model" library_version="11">
<description>CECC Size RC2211 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204R" urn="urn:adsk.eagle:package:26109/2" type="model" library_version="11">
<description>CECC Size RC3715 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204W" urn="urn:adsk.eagle:package:26111/2" type="model" library_version="11">
<description>CECC Size RC3715 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207R" urn="urn:adsk.eagle:package:26113/2" type="model" library_version="11">
<description>CECC Size RC6123 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207W" urn="urn:adsk.eagle:package:26112/2" type="model" library_version="11">
<description>CECC Size RC6123 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207W"/>
</packageinstances>
</package3d>
<package3d name="R0201" urn="urn:adsk.eagle:package:26117/2" type="model" library_version="11">
<description>RESISTOR chip
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R0201"/>
</packageinstances>
</package3d>
<package3d name="VTA52" urn="urn:adsk.eagle:package:26116/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR52
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA52"/>
</packageinstances>
</package3d>
<package3d name="VTA53" urn="urn:adsk.eagle:package:26118/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR53
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA53"/>
</packageinstances>
</package3d>
<package3d name="VTA54" urn="urn:adsk.eagle:package:26119/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR54
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA54"/>
</packageinstances>
</package3d>
<package3d name="VTA55" urn="urn:adsk.eagle:package:26120/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA55"/>
</packageinstances>
</package3d>
<package3d name="VTA56" urn="urn:adsk.eagle:package:26129/3" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR56
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA56"/>
</packageinstances>
</package3d>
<package3d name="VMTA55" urn="urn:adsk.eagle:package:26121/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTA55"/>
</packageinstances>
</package3d>
<package3d name="VMTB60" urn="urn:adsk.eagle:package:26122/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC60
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTB60"/>
</packageinstances>
</package3d>
<package3d name="R4527" urn="urn:adsk.eagle:package:13310/2" type="model" library_version="11">
<description>Package 4527
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<packageinstances>
<packageinstance name="R4527"/>
</packageinstances>
</package3d>
<package3d name="WSC0001" urn="urn:adsk.eagle:package:26123/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0001"/>
</packageinstances>
</package3d>
<package3d name="WSC0002" urn="urn:adsk.eagle:package:26125/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0002"/>
</packageinstances>
</package3d>
<package3d name="WSC01/2" urn="urn:adsk.eagle:package:26127/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC01/2"/>
</packageinstances>
</package3d>
<package3d name="WSC2515" urn="urn:adsk.eagle:package:26134/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC2515"/>
</packageinstances>
</package3d>
<package3d name="WSC4527" urn="urn:adsk.eagle:package:26126/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC4527"/>
</packageinstances>
</package3d>
<package3d name="WSC6927" urn="urn:adsk.eagle:package:26128/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC6927"/>
</packageinstances>
</package3d>
<package3d name="R1218" urn="urn:adsk.eagle:package:26131/2" type="model" library_version="11">
<description>CRCW1218 Thick Film, Rectangular Chip Resistors
Source: http://www.vishay.com .. dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R1218"/>
</packageinstances>
</package3d>
<package3d name="1812X7R" urn="urn:adsk.eagle:package:26130/2" type="model" library_version="11">
<description>Chip Monolithic Ceramic Capacitors Medium Voltage High Capacitance for General Use
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<packageinstances>
<packageinstance name="1812X7R"/>
</packageinstances>
</package3d>
<package3d name="PRL1632" urn="urn:adsk.eagle:package:26132/2" type="model" library_version="11">
<description>PRL1632 are realized as 1W for 3.2 × 1.6mm(1206)
Source: http://www.mouser.com/ds/2/392/products_18-2245.pdf</description>
<packageinstances>
<packageinstance name="PRL1632"/>
</packageinstances>
</package3d>
<package3d name="R01005" urn="urn:adsk.eagle:package:26133/2" type="model" library_version="11">
<description>Chip, 0.40 X 0.20 X 0.16 mm body
&lt;p&gt;Chip package with body size 0.40 X 0.20 X 0.16 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="R01005"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="R-EU" urn="urn:adsk.eagle:symbol:23042/1" library_version="5">
<wire x1="-2.54" y1="-0.889" x2="2.54" y2="-0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="0.889" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="-0.889" x2="2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.889" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="R-EU_" urn="urn:adsk.eagle:component:23791/21" prefix="R" uservalue="yes" library_version="11">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, European symbol</description>
<gates>
<gate name="G$1" symbol="R-EU" x="0" y="0"/>
</gates>
<devices>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23547/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23555/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="70" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0805" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23553/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="86" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0805W" package="R0805W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23537/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23540/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="41" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1206W" package="R1206W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23539/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23554/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1210W" package="R1210W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23541/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23551/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2010W" package="R2010W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23542/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23543/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2012W" package="R2012W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23544/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23545/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2512W" package="R2512W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23565/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23557/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3216W" package="R3216W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23548/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="5" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23549/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3225W" package="R3225W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23550/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23552/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R5025W" package="R5025W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23558/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23559/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R6332W" package="R6332W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26078/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M0805" package="M0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23556/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="45" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M1206" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23566/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M1406" package="M1406">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23569/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M2012" package="M2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23561/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M2309" package="M2309">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23562/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M3216" package="M3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23563/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M3516" package="M3516">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23573/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M5923" package="M5923">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23564/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/5" package="0204/5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23488/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="35" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23498/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="79" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/2V" package="0204V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23495/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="11" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23491/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="81" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/12" package="0207/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23489/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="9" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/15" package="0207/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23492/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/2V" package="0207/2V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23490/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/5V" package="0207/5V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23502/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="4" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/7" package="0207/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23493/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="46" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/10" package="0309/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23567/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/12" package="0309/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23571/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="6" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/V" package="0309V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23572/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/12" package="0411/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23578/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="5" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/15" package="0411/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23568/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/3V" package="0411V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23570/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="4" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0414/15" package="0414/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23579/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0414/5V" package="0414V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23574/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/17" package="0617/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23575/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/22" package="0617/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23577/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/5V" package="0617V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23576/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0922/22" package="0922/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23580/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0613/5V" package="P0613V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23582/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0613/15" package="P0613/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23581/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0817/22" package="P0817/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23583/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0817/7V" package="P0817V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23608/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V234/12" package="V234/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23592/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V235/17" package="V235/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23586/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V526-0" package="V526-0">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23590/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102R" package="MINI_MELF-0102R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23591/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102W" package="MINI_MELF-0102W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23588/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0204R" package="MINI_MELF-0204R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26109/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0204W" package="MINI_MELF-0204W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26111/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0207R" package="MINI_MELF-0207R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26113/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0207W" package="MINI_MELF-0207W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26112/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0922V" package="0922V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23589/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="RDH/15" package="RDH/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23595/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102AX" package="MINI_MELF-0102AX">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23594/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0201" package="R0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26117/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA52" package="VTA52">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26116/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA53" package="VTA53">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26118/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA54" package="VTA54">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26119/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA55" package="VTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26120/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA56" package="VTA56">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26129/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VMTA55" package="VMTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26121/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VMTB60" package="VMTB60">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26122/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R4527" package="R4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13310/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC0001" package="WSC0001">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26123/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC0002" package="WSC0002">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26125/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC01/2" package="WSC01/2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26127/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC2515" package="WSC2515">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26134/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC4527" package="WSC4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26126/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC6927" package="WSC6927">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26128/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1218" package="R1218">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26131/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="1812X7R" package="1812X7R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26130/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="PRL1632" package="PRL1632">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26132/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="01005" package="R01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26133/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
</devices>
<spice>
<pinmapping spiceprefix="R">
<pinmap gate="G$1" pin="1" pinorder="1"/>
<pinmap gate="G$1" pin="2" pinorder="2"/>
</pinmapping>
</spice>
</deviceset>
</devicesets>
</library>
<library name="led" urn="urn:adsk.eagle:library:259">
<description>&lt;b&gt;LEDs&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;br&gt;
Extended by Federico Battaglin &lt;author&gt;&amp;lt;federico.rd@fdpinternational.com&amp;gt;&lt;/author&gt; with DUOLED</description>
<packages>
<package name="1206" urn="urn:adsk.eagle:footprint:15651/1" library_version="4">
<description>&lt;b&gt;CHICAGO MINIATURE LAMP, INC.&lt;/b&gt;&lt;p&gt;
7022X Series SMT LEDs 1206 Package Size</description>
<wire x1="1.55" y1="-0.75" x2="-1.55" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.55" y1="-0.75" x2="-1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.55" y1="0.75" x2="1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.55" y1="0.75" x2="1.55" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-0.55" y1="-0.5" x2="0.55" y2="-0.5" width="0.1016" layer="21" curve="95.452622"/>
<wire x1="-0.55" y1="-0.5" x2="-0.55" y2="0.5" width="0.1016" layer="51" curve="-84.547378"/>
<wire x1="-0.55" y1="0.5" x2="0.55" y2="0.5" width="0.1016" layer="21" curve="-95.452622"/>
<wire x1="0.55" y1="0.5" x2="0.55" y2="-0.5" width="0.1016" layer="51" curve="-84.547378"/>
<smd name="A" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="C" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.1" y1="-0.1" x2="0.1" y2="0.1" layer="21"/>
<rectangle x1="0.45" y1="-0.7" x2="0.8" y2="-0.45" layer="51"/>
<rectangle x1="0.8" y1="-0.7" x2="0.9" y2="0.5" layer="51"/>
<rectangle x1="0.8" y1="0.55" x2="0.9" y2="0.7" layer="51"/>
<rectangle x1="-0.9" y1="-0.7" x2="-0.8" y2="0.5" layer="51"/>
<rectangle x1="-0.9" y1="0.55" x2="-0.8" y2="0.7" layer="51"/>
<rectangle x1="0.45" y1="-0.7" x2="0.6" y2="-0.45" layer="21"/>
</package>
<package name="LD260" urn="urn:adsk.eagle:footprint:15652/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
5 mm, square, Siemens</description>
<wire x1="-1.27" y1="-1.27" x2="0" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.889" x2="1.27" y2="0" width="0.1524" layer="51"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.889" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="0.889" width="0.1524" layer="51"/>
<wire x1="0" y1="1.27" x2="0.9917" y2="0.7934" width="0.1524" layer="21" curve="-51.33923"/>
<wire x1="-0.9917" y1="0.7934" x2="0" y2="1.27" width="0.1524" layer="21" curve="-51.33923"/>
<wire x1="0" y1="-1.27" x2="0.9917" y2="-0.7934" width="0.1524" layer="21" curve="51.33923"/>
<wire x1="-0.9917" y1="-0.7934" x2="0" y2="-1.27" width="0.1524" layer="21" curve="51.33923"/>
<wire x1="0.9558" y1="-0.8363" x2="1.27" y2="0" width="0.1524" layer="51" curve="41.185419"/>
<wire x1="0.9756" y1="0.813" x2="1.2699" y2="0" width="0.1524" layer="51" curve="-39.806332"/>
<wire x1="-1.27" y1="0" x2="-0.9643" y2="-0.8265" width="0.1524" layer="51" curve="40.600331"/>
<wire x1="-1.27" y1="0" x2="-0.9643" y2="0.8265" width="0.1524" layer="51" curve="-40.600331"/>
<wire x1="-0.889" y1="0" x2="0" y2="0.889" width="0.1524" layer="51" curve="-90"/>
<wire x1="-0.508" y1="0" x2="0" y2="0.508" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.508" x2="0.508" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="0" y1="-0.889" x2="0.889" y2="0" width="0.1524" layer="51" curve="90"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="1.4732" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-2.4892" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="1.27" y1="-0.635" x2="2.032" y2="0.635" layer="51"/>
<rectangle x1="1.905" y1="-0.635" x2="2.032" y2="0.635" layer="21"/>
</package>
<package name="LED2X5" urn="urn:adsk.eagle:footprint:15653/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
2 x 5 mm, rectangle</description>
<wire x1="-2.54" y1="-1.27" x2="2.54" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.27" x2="-2.54" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.27" x2="-2.54" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="0" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.381" x2="-0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.381" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="-0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0.381" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.889" y1="-0.254" x2="1.143" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.762" x2="1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.762" x2="0.9398" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="0.9398" y1="-0.6096" x2="1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.397" y1="-0.254" x2="1.651" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-0.762" x2="1.651" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-0.762" x2="1.4478" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="1.4478" y1="-0.6096" x2="1.651" y2="-0.508" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-2.413" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.159" y1="-1.27" x2="2.413" y2="1.27" layer="21"/>
</package>
<package name="LED3MM" urn="urn:adsk.eagle:footprint:15654/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
3 mm, round</description>
<wire x1="1.5748" y1="-1.27" x2="1.5748" y2="1.27" width="0.254" layer="51"/>
<wire x1="-1.524" y1="0" x2="-1.1708" y2="0.9756" width="0.1524" layer="51" curve="-39.80361"/>
<wire x1="-1.524" y1="0" x2="-1.1391" y2="-1.0125" width="0.1524" layer="51" curve="41.633208"/>
<wire x1="1.1571" y1="0.9918" x2="1.524" y2="0" width="0.1524" layer="51" curve="-40.601165"/>
<wire x1="1.1708" y1="-0.9756" x2="1.524" y2="0" width="0.1524" layer="51" curve="39.80361"/>
<wire x1="0" y1="1.524" x2="1.2401" y2="0.8858" width="0.1524" layer="21" curve="-54.461337"/>
<wire x1="-1.2192" y1="0.9144" x2="0" y2="1.524" width="0.1524" layer="21" curve="-53.130102"/>
<wire x1="0" y1="-1.524" x2="1.203" y2="-0.9356" width="0.1524" layer="21" curve="52.126876"/>
<wire x1="-1.203" y1="-0.9356" x2="0" y2="-1.524" width="0.1524" layer="21" curve="52.126876"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0" y1="-1.016" x2="1.016" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0" y1="2.032" x2="1.561" y2="1.3009" width="0.254" layer="21" curve="-50.193108"/>
<wire x1="-1.7929" y1="0.9562" x2="0" y2="2.032" width="0.254" layer="21" curve="-61.926949"/>
<wire x1="0" y1="-2.032" x2="1.5512" y2="-1.3126" width="0.254" layer="21" curve="49.763022"/>
<wire x1="-1.7643" y1="-1.0082" x2="0" y2="-2.032" width="0.254" layer="21" curve="60.255215"/>
<wire x1="-2.032" y1="0" x2="-1.7891" y2="0.9634" width="0.254" layer="51" curve="-28.301701"/>
<wire x1="-2.032" y1="0" x2="-1.7306" y2="-1.065" width="0.254" layer="51" curve="31.60822"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="1.905" y="0.381" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="1.905" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="LED5MM" urn="urn:adsk.eagle:footprint:15655/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
5 mm, round</description>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.254" layer="21" curve="-286.260205"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.159" y1="0" x2="0" y2="2.159" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-2.159" x2="2.159" y2="0" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0.5334" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.2004" y="-1.8034" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="LSU260" urn="urn:adsk.eagle:footprint:15656/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
1 mm, round, Siemens</description>
<wire x1="0" y1="-0.508" x2="-1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.508" x2="-1.143" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="0.508" x2="0" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.254" x2="-1.397" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.254" x2="-1.143" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="-0.254" x2="-1.397" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="0.254" x2="-1.143" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="0.254" x2="-1.143" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-0.254" x2="1.397" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="1.397" y1="-0.254" x2="1.397" y2="0.254" width="0.1524" layer="51"/>
<wire x1="1.397" y1="0.254" x2="0.508" y2="0.254" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.381" x2="0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.254" y1="-0.508" x2="-0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="-0.381" x2="-0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.381" y1="0.381" x2="0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0.508" x2="-0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0.381" x2="-0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.254" x2="0.254" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.254" y1="0" x2="0" y2="0.254" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.381" y1="-0.381" x2="0.381" y2="0.381" width="0.1524" layer="21" curve="90"/>
<circle x="0" y="0" radius="0.508" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="0.8382" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-1.8542" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.397" y1="-0.254" x2="-1.143" y2="0.254" layer="51"/>
<rectangle x1="0.508" y1="-0.254" x2="1.397" y2="0.254" layer="51"/>
</package>
<package name="LZR181" urn="urn:adsk.eagle:footprint:15657/1" library_version="4">
<description>&lt;B&gt;LED BLOCK&lt;/B&gt;&lt;p&gt;
1 LED, Siemens</description>
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.889" x2="1.27" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.889" x2="-1.27" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="0" x2="0" y2="0.889" width="0.1524" layer="51" curve="-90"/>
<wire x1="-0.508" y1="0" x2="0" y2="0.508" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.508" x2="0.508" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="0" y1="-0.889" x2="0.889" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-0.8678" y1="0.7439" x2="0" y2="1.143" width="0.1524" layer="21" curve="-49.396139"/>
<wire x1="0" y1="1.143" x2="0.8678" y2="0.7439" width="0.1524" layer="21" curve="-49.396139"/>
<wire x1="-0.8678" y1="-0.7439" x2="0" y2="-1.143" width="0.1524" layer="21" curve="49.396139"/>
<wire x1="0" y1="-1.143" x2="0.8678" y2="-0.7439" width="0.1524" layer="21" curve="49.396139"/>
<wire x1="0.8678" y1="0.7439" x2="1.143" y2="0" width="0.1524" layer="51" curve="-40.604135"/>
<wire x1="0.8678" y1="-0.7439" x2="1.143" y2="0" width="0.1524" layer="51" curve="40.604135"/>
<wire x1="-1.143" y1="0" x2="-0.8678" y2="0.7439" width="0.1524" layer="51" curve="-40.604135"/>
<wire x1="-1.143" y1="0" x2="-0.8678" y2="-0.7439" width="0.1524" layer="51" curve="40.604135"/>
<wire x1="-1.27" y1="1.27" x2="1.27" y2="1.27" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="1.4732" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-2.4892" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="1.27" y1="-0.889" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.27" y2="0.254" layer="51"/>
</package>
<package name="Q62902-B152" urn="urn:adsk.eagle:footprint:15658/1" library_version="4">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-2.9718" y1="-1.8542" x2="-2.9718" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="-0.254" x2="-2.9718" y2="0.254" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="0.254" x2="-2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="2.9718" y1="-1.8542" x2="-2.1082" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.1082" y1="-1.8542" x2="-2.54" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.8542" x2="-2.1082" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.1082" y1="1.8542" x2="2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.8542" x2="-2.54" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.8542" x2="-2.9718" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.8542" x2="-2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="0.254" x2="-2.9718" y2="-0.254" width="0.1524" layer="21" curve="180"/>
<wire x1="-1.1486" y1="0.8814" x2="0" y2="1.4478" width="0.1524" layer="21" curve="-52.498642"/>
<wire x1="0" y1="1.4478" x2="1.1476" y2="0.8827" width="0.1524" layer="21" curve="-52.433716"/>
<wire x1="-1.1351" y1="-0.8987" x2="0" y2="-1.4478" width="0.1524" layer="21" curve="51.629985"/>
<wire x1="0" y1="-1.4478" x2="1.1305" y2="-0.9044" width="0.1524" layer="21" curve="51.339172"/>
<wire x1="1.1281" y1="-0.9074" x2="1.4478" y2="0" width="0.1524" layer="51" curve="38.811177"/>
<wire x1="1.1401" y1="0.8923" x2="1.4478" y2="0" width="0.1524" layer="51" curve="-38.048073"/>
<wire x1="-1.4478" y1="0" x2="-1.1305" y2="-0.9044" width="0.1524" layer="51" curve="38.659064"/>
<wire x1="-1.4478" y1="0" x2="-1.1456" y2="0.8853" width="0.1524" layer="51" curve="-37.696376"/>
<wire x1="0" y1="1.7018" x2="1.4674" y2="0.8618" width="0.1524" layer="21" curve="-59.573488"/>
<wire x1="-1.4618" y1="0.8714" x2="0" y2="1.7018" width="0.1524" layer="21" curve="-59.200638"/>
<wire x1="0" y1="-1.7018" x2="1.4571" y2="-0.8793" width="0.1524" layer="21" curve="58.891781"/>
<wire x1="-1.4571" y1="-0.8793" x2="0" y2="-1.7018" width="0.1524" layer="21" curve="58.891781"/>
<wire x1="-1.7018" y1="0" x2="-1.4447" y2="0.8995" width="0.1524" layer="51" curve="-31.907626"/>
<wire x1="-1.7018" y1="0" x2="-1.4502" y2="-0.8905" width="0.1524" layer="51" curve="31.551992"/>
<wire x1="1.4521" y1="0.8874" x2="1.7018" y2="0" width="0.1524" layer="51" curve="-31.429586"/>
<wire x1="1.4459" y1="-0.8975" x2="1.7018" y2="0" width="0.1524" layer="51" curve="31.828757"/>
<wire x1="-2.1082" y1="1.8542" x2="-2.1082" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<wire x1="2.9718" y1="1.8542" x2="2.9718" y2="-1.8542" width="0.1524" layer="21"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.905" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.905" y="-3.556" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="Q62902-B153" urn="urn:adsk.eagle:footprint:15659/1" library_version="4">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-5.5118" y1="-3.5052" x2="-5.5118" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="-0.254" x2="-5.5118" y2="0.254" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="0.254" x2="-5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="5.5118" y1="-3.5052" x2="-4.6482" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-4.6482" y1="-3.5052" x2="-5.08" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.5052" x2="-4.6482" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-4.6482" y1="3.5052" x2="5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-3.5052" x2="-5.08" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-3.5052" x2="-5.5118" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.5052" x2="-5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="0.254" x2="-5.5118" y2="-0.254" width="0.1524" layer="21" curve="180"/>
<wire x1="-4.6482" y1="3.5052" x2="-4.6482" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="5.5118" y1="3.5052" x2="5.5118" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.905" x2="-2.54" y2="-1.905" width="0.254" layer="21"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-2.159" x2="2.159" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.2129" y1="0.0539" x2="-0.0539" y2="2.2129" width="0.1524" layer="51" curve="-90.010616"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.175" width="0.254" layer="21"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-4.191" y="3.937" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-5.08" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="Q62902-B155" urn="urn:adsk.eagle:footprint:15660/1" library_version="4">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-1.27" y1="-3.048" x2="-1.27" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="10.033" y1="3.048" x2="2.921" y2="3.048" width="0.1524" layer="21"/>
<wire x1="10.033" y1="3.048" x2="10.033" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-3.048" x2="2.921" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-3.048" x2="2.921" y2="3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-3.048" x2="10.033" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="3.048" x2="-1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="2.54" x2="-5.207" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="2.54" x2="-1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.54" x2="-1.27" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-2.54" x2="-1.27" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.54" x2="-5.207" y2="-2.54" width="0.1524" layer="21" curve="180"/>
<wire x1="-6.985" y1="0.635" x2="-6.985" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="1.397" x2="-6.096" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="1.905" x2="-5.207" y2="-1.905" width="0.1524" layer="21"/>
<pad name="K" x="7.62" y="1.27" drill="0.8128" shape="long"/>
<pad name="A" x="7.62" y="-1.27" drill="0.8128" shape="long"/>
<text x="3.302" y="-2.794" size="1.016" layer="21" ratio="14">A+</text>
<text x="3.302" y="1.778" size="1.016" layer="21" ratio="14">K-</text>
<text x="11.684" y="-2.794" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="0.635" y="-4.445" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.921" y1="1.016" x2="6.731" y2="1.524" layer="21"/>
<rectangle x1="2.921" y1="-1.524" x2="6.731" y2="-1.016" layer="21"/>
<hole x="0" y="0" drill="0.8128"/>
</package>
<package name="Q62902-B156" urn="urn:adsk.eagle:footprint:15661/1" library_version="4">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.159" y1="0" x2="0" y2="2.159" width="0.1524" layer="51" curve="-90"/>
<wire x1="0.0539" y1="-2.2129" x2="2.2129" y2="-0.0539" width="0.1524" layer="51" curve="90.005308"/>
<wire x1="2.54" y1="3.81" x2="3.81" y2="2.54" width="0.1524" layer="21"/>
<wire x1="2.54" y1="3.81" x2="-3.81" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-3.81" x2="-3.81" y2="3.81" width="0.1524" layer="21"/>
<wire x1="3.81" y1="2.54" x2="3.81" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-3.81" x2="-2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-3.302" x2="-2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-3.81" x2="2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-3.302" x2="2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-3.302" x2="-2.54" y2="-3.302" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.175" width="0.254" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-3.81" y="4.0894" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.7846" y="-5.3594" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.556" y="-3.302" size="1.016" layer="21" ratio="14">+</text>
<text x="2.794" y="-3.302" size="1.016" layer="21" ratio="14">-</text>
</package>
<package name="SFH480" urn="urn:adsk.eagle:footprint:15662/1" library_version="4">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking&lt;p&gt;
Inifineon</description>
<wire x1="-2.159" y1="1.524" x2="-2.794" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0" y1="1.778" x2="1.5358" y2="0.8959" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="0.8959" x2="0" y2="1.778" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="-0.8959" x2="0" y2="-1.778" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="0" y1="-1.778" x2="1.5358" y2="-0.8959" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="1.5142" y1="0.9318" x2="1.778" y2="0" width="0.1524" layer="51" curve="-31.606487"/>
<wire x1="1.5" y1="-0.9546" x2="1.778" y2="0" width="0.1524" layer="51" curve="32.472615"/>
<wire x1="-1.778" y1="0" x2="-1.5142" y2="-0.9318" width="0.1524" layer="51" curve="31.606487"/>
<wire x1="-1.778" y1="0" x2="-1.5" y2="0.9546" width="0.1524" layer="51" curve="-32.472615"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.667" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.413" width="0.254" layer="21"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.27" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.318" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SFH482" urn="urn:adsk.eagle:footprint:15650/1" library_version="4">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking&lt;p&gt;
Inifineon</description>
<wire x1="-2.159" y1="1.524" x2="-2.794" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0" y1="1.778" x2="1.5358" y2="0.8959" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="0.8959" x2="0" y2="1.778" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="-0.8959" x2="0" y2="-1.778" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="0" y1="-1.778" x2="1.5358" y2="-0.8959" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="1.5142" y1="0.9318" x2="1.778" y2="0" width="0.1524" layer="51" curve="-31.606487"/>
<wire x1="1.5" y1="-0.9546" x2="1.778" y2="0" width="0.1524" layer="51" curve="32.472615"/>
<wire x1="-1.778" y1="0" x2="-1.5142" y2="-0.9318" width="0.1524" layer="51" curve="31.606487"/>
<wire x1="-1.778" y1="0" x2="-1.5" y2="0.9546" width="0.1524" layer="51" curve="-32.472615"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.667" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.413" width="0.254" layer="21"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.27" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.318" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="U57X32" urn="urn:adsk.eagle:footprint:15640/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
rectangle, 5.7 x 3.2 mm</description>
<wire x1="-3.175" y1="1.905" x2="3.175" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.905" x2="3.175" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.905" x2="-3.175" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.905" x2="-3.175" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="1.397" x2="2.667" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.667" y1="-1.397" x2="2.667" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.667" y1="-1.397" x2="-2.667" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="1.397" x2="-2.667" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="2.54" y2="1.016" width="0.1524" layer="51"/>
<wire x1="2.286" y1="1.27" x2="2.286" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="2.54" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="2.54" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-1.016" x2="2.54" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="1.27" x2="-1.778" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-0.762" y1="1.27" x2="-0.762" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="1.27" x2="-0.254" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="0.254" y1="1.27" x2="0.254" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="0.762" y1="1.27" x2="0.762" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="1.778" y1="1.27" x2="1.778" y2="-1.27" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="3.683" y="0.254" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.683" y="-1.524" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="IRL80A" urn="urn:adsk.eagle:footprint:15663/1" library_version="4">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
IR transmitter Siemens</description>
<wire x1="0.889" y1="2.286" x2="0.889" y2="1.778" width="0.1524" layer="21"/>
<wire x1="0.889" y1="1.778" x2="0.889" y2="0.762" width="0.1524" layer="51"/>
<wire x1="0.889" y1="0.762" x2="0.889" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.889" y1="-0.635" x2="0.889" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="0.889" y1="-1.778" x2="0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="0.889" y1="-2.286" x2="-0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="2.286" x2="-0.889" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="1.778" x2="-0.889" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="0.762" x2="-0.889" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="-0.762" x2="-0.889" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="-1.778" x2="-0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="2.286" x2="0.889" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="-0.762" x2="-0.889" y2="0.762" width="0.1524" layer="21" curve="-180"/>
<wire x1="-1.397" y1="0.254" x2="-1.397" y2="-0.254" width="0.0508" layer="21"/>
<wire x1="-1.143" y1="0.508" x2="-1.143" y2="-0.508" width="0.0508" layer="21"/>
<pad name="K" x="0" y="1.27" drill="0.8128" shape="octagon"/>
<pad name="A" x="0" y="-1.27" drill="0.8128" shape="octagon"/>
<text x="1.27" y="0.381" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="1.27" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="P-LCC-2" urn="urn:adsk.eagle:footprint:15664/1" library_version="4">
<description>&lt;b&gt;TOPLED® High-optical Power LED (HOP)&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... ls_t675.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="1.8" x2="1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="1.8" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-1.1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.8" x2="1.1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="-1.8" x2="1.1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="51"/>
<smd name="C" x="0" y="-2.75" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.75" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-0.635" y="2.54" size="1.27" layer="21">A</text>
<text x="-0.635" y="-3.81" size="1.27" layer="21">C</text>
<rectangle x1="-1.3" y1="-2.25" x2="1.3" y2="-0.75" layer="31"/>
<rectangle x1="-1.3" y1="0.75" x2="1.3" y2="2.25" layer="31"/>
<rectangle x1="-0.25" y1="-0.25" x2="0.25" y2="0.25" layer="21"/>
<rectangle x1="-1.4" y1="0.65" x2="1.4" y2="2.35" layer="29"/>
<rectangle x1="-1.4" y1="-2.35" x2="1.4" y2="-0.65" layer="29"/>
</package>
<package name="OSRAM-MINI-TOP-LED" urn="urn:adsk.eagle:footprint:15665/1" library_version="4">
<description>&lt;b&gt;BLUE LINETM Hyper Mini TOPLED® Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LB M676.pdf</description>
<wire x1="-0.6" y1="0.9" x2="-0.6" y2="-0.7" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-0.9" x2="-0.4" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="-0.4" y1="-0.9" x2="0.6" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="0.6" y1="-0.9" x2="0.6" y2="0.9" width="0.1016" layer="51"/>
<wire x1="0.6" y1="0.9" x2="-0.6" y2="0.9" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="0.95" x2="-0.45" y2="1.1" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="1.1" x2="0.45" y2="1.1" width="0.1016" layer="51"/>
<wire x1="0.45" y1="1.1" x2="0.45" y2="0.95" width="0.1016" layer="51"/>
<wire x1="-0.6" y1="-0.7" x2="-0.4" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-0.9" x2="-0.45" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-1.1" x2="0.45" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="0.45" y1="-1.1" x2="0.45" y2="-0.95" width="0.1016" layer="51"/>
<smd name="A" x="0" y="2.6" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="C" x="0" y="-2.6" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-0.635" y="1.905" size="1.27" layer="21">A</text>
<text x="-0.635" y="-3.175" size="1.27" layer="21">C</text>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.5" y1="0.6" x2="0.5" y2="1.4" layer="29"/>
<rectangle x1="-0.5" y1="-1.4" x2="0.5" y2="-0.6" layer="29"/>
<rectangle x1="-0.15" y1="-0.6" x2="0.15" y2="-0.3" layer="51"/>
<rectangle x1="-0.45" y1="0.65" x2="0.45" y2="1.35" layer="31"/>
<rectangle x1="-0.45" y1="-1.35" x2="0.45" y2="-0.65" layer="31"/>
</package>
<package name="OSRAM-SIDELED" urn="urn:adsk.eagle:footprint:15666/1" library_version="4">
<description>&lt;b&gt;Super SIDELED® High-Current LED&lt;/b&gt;&lt;p&gt;
LG A672, LP A672 &lt;br&gt;
Source: http://www.osram.convergy.de/ ... LG_LP_A672.pdf (2004.05.13)</description>
<wire x1="-1.85" y1="-2.05" x2="-1.85" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="-0.75" x2="-1.7" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.7" y1="-0.75" x2="-1.7" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.7" y1="0.75" x2="-1.85" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="0.75" x2="-1.85" y2="2.05" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="2.05" x2="0.9" y2="2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="2.05" x2="0.9" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="-2.05" x2="-1.85" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="-2.05" x2="1.05" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="-2.05" x2="1.85" y2="-1.85" width="0.1016" layer="51"/>
<wire x1="1.85" y1="-1.85" x2="1.85" y2="1.85" width="0.1016" layer="51"/>
<wire x1="1.85" y1="1.85" x2="1.05" y2="2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="2.05" x2="0.9" y2="2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="2.05" x2="1.05" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="-0.55" y1="-0.9" x2="-0.55" y2="0.9" width="0.1016" layer="51" curve="-167.319617"/>
<wire x1="-0.55" y1="-0.9" x2="0.85" y2="-1.2" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.55" y1="0.9" x2="0.85" y2="1.2" width="0.1016" layer="51" style="shortdash"/>
<smd name="C" x="0" y="-2.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="0.635" y="-3.175" size="1.27" layer="21" rot="R90">C</text>
<text x="0.635" y="2.54" size="1.27" layer="21" rot="R90">A</text>
<text x="-2.54" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.1" y1="-2.2" x2="2.1" y2="-0.4" layer="29"/>
<rectangle x1="-2.1" y1="0.4" x2="2.1" y2="2.2" layer="29"/>
<rectangle x1="-1.9" y1="-2.1" x2="1.9" y2="-0.6" layer="31"/>
<rectangle x1="-1.9" y1="0.6" x2="1.9" y2="2.1" layer="31"/>
<rectangle x1="-1.85" y1="-2.05" x2="-0.7" y2="-1" layer="51"/>
</package>
<package name="SMART-LED" urn="urn:adsk.eagle:footprint:15667/1" library_version="4">
<description>&lt;b&gt;SmartLEDTM Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY L896.pdf</description>
<wire x1="-0.35" y1="0.6" x2="0.35" y2="0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.35" y1="0.6" x2="0.35" y2="-0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.6" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.15" y1="-0.6" x2="-0.35" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.35" y1="-0.6" x2="-0.35" y2="0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.4" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<smd name="A" x="0" y="0.725" dx="0.35" dy="0.35" layer="1"/>
<smd name="B" x="0" y="-0.725" dx="0.35" dy="0.35" layer="1"/>
<text x="-0.635" y="-0.635" size="1.016" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-0.635" size="1.016" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.15" y1="-0.35" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.15" y1="0.6" x2="0.15" y2="0.85" layer="51"/>
<rectangle x1="-0.15" y1="-0.85" x2="0.15" y2="-0.6" layer="51"/>
</package>
<package name="P-LCC-2-TOPLED-RG" urn="urn:adsk.eagle:footprint:15668/1" library_version="4">
<description>&lt;b&gt;Hyper TOPLED® RG Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY T776.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.1" y2="2.45" width="0.1016" layer="51"/>
<wire x1="1.1" y1="2.45" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-1.1" y2="-2.45" width="0.1016" layer="51"/>
<wire x1="1.1" y1="-2.45" x2="1.1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="21"/>
<smd name="C" x="0" y="-3.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="3.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-0.635" y="3.29" size="1.27" layer="21">A</text>
<text x="-0.635" y="-4.56" size="1.27" layer="21">C</text>
<rectangle x1="-1.3" y1="-3" x2="1.3" y2="-1.5" layer="31"/>
<rectangle x1="-1.3" y1="1.5" x2="1.3" y2="3" layer="31"/>
<rectangle x1="-0.25" y1="-0.25" x2="0.25" y2="0.25" layer="21"/>
<rectangle x1="-1.15" y1="2.4" x2="1.15" y2="2.7" layer="51"/>
<rectangle x1="-1.15" y1="-2.7" x2="1.15" y2="-2.4" layer="51"/>
<rectangle x1="-1.5" y1="1.5" x2="1.5" y2="3.2" layer="29"/>
<rectangle x1="-1.5" y1="-3.2" x2="1.5" y2="-1.5" layer="29"/>
<hole x="0" y="0" drill="2.8"/>
</package>
<package name="MICRO-SIDELED" urn="urn:adsk.eagle:footprint:15669/1" library_version="4">
<description>&lt;b&gt;Hyper Micro SIDELED®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY Y876.pdf</description>
<wire x1="0.65" y1="1.1" x2="-0.1" y2="1.1" width="0.1016" layer="51"/>
<wire x1="-0.1" y1="1.1" x2="-0.35" y2="1" width="0.1016" layer="51"/>
<wire x1="-0.35" y1="1" x2="-0.35" y2="-0.9" width="0.1016" layer="21"/>
<wire x1="-0.35" y1="-0.9" x2="-0.1" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="-0.1" y1="-1.1" x2="0.65" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="0.65" y1="-1.1" x2="0.65" y2="1.1" width="0.1016" layer="21"/>
<wire x1="0.6" y1="0.9" x2="0.25" y2="0.7" width="0.0508" layer="21"/>
<wire x1="0.25" y1="0.7" x2="0.25" y2="-0.7" width="0.0508" layer="21"/>
<wire x1="0.25" y1="-0.7" x2="0.6" y2="-0.9" width="0.0508" layer="21"/>
<smd name="A" x="0" y="1.95" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<smd name="C" x="0" y="-1.95" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<text x="-1.27" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.4" y1="1.1" x2="0.4" y2="1.8" layer="29"/>
<rectangle x1="-0.4" y1="-1.8" x2="0.4" y2="-1.1" layer="29"/>
<rectangle x1="-0.35" y1="-1.75" x2="0.35" y2="-1.15" layer="31"/>
<rectangle x1="-0.35" y1="1.15" x2="0.35" y2="1.75" layer="31"/>
<rectangle x1="-0.125" y1="1.125" x2="0.125" y2="1.75" layer="51"/>
<rectangle x1="-0.125" y1="-1.75" x2="0.125" y2="-1.125" layer="51"/>
</package>
<package name="P-LCC-4" urn="urn:adsk.eagle:footprint:15670/1" library_version="4">
<description>&lt;b&gt;Power TOPLED®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LA_LY E67B.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="1.6" x2="-1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-1" y1="1.8" x2="-0.5" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-0.5" y1="1.8" x2="-0.5" y2="1.65" width="0.1016" layer="51"/>
<wire x1="0.5" y1="1.65" x2="0.5" y2="1.8" width="0.1016" layer="51"/>
<wire x1="0.5" y1="1.8" x2="1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="1.8" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1" y1="-1.6" x2="-1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-1" y1="-1.8" x2="-0.5" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-0.5" y1="-1.8" x2="-0.5" y2="-1.65" width="0.1016" layer="51"/>
<wire x1="0.5" y1="-1.65" x2="0.5" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="0.5" y1="-1.8" x2="1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="1" y1="-1.8" x2="1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="51"/>
<smd name="A" x="-2" y="3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@3" x="2" y="3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@4" x="2" y="-3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@1" x="-2" y="-3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<text x="-3.81" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="5.08" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-1.905" y="-3.81" size="1.27" layer="21">C</text>
<text x="-1.905" y="2.54" size="1.27" layer="21">A</text>
<text x="1.27" y="2.54" size="1.27" layer="21">C</text>
<text x="1.27" y="-3.81" size="1.27" layer="21">C</text>
<rectangle x1="-1.15" y1="0.75" x2="-0.35" y2="1.85" layer="29"/>
<rectangle x1="0.35" y1="0.75" x2="1.15" y2="1.85" layer="29"/>
<rectangle x1="0.35" y1="-1.85" x2="1.15" y2="-0.75" layer="29"/>
<rectangle x1="-1.15" y1="-1.85" x2="-0.35" y2="-0.75" layer="29"/>
<rectangle x1="-1.1" y1="-1.8" x2="-0.4" y2="-0.8" layer="31"/>
<rectangle x1="0.4" y1="-1.8" x2="1.1" y2="-0.8" layer="31"/>
<rectangle x1="0.4" y1="0.8" x2="1.1" y2="1.8" layer="31"/>
<rectangle x1="-1.1" y1="0.8" x2="-0.4" y2="1.8" layer="31"/>
<rectangle x1="-0.2" y1="-0.2" x2="0.2" y2="0.2" layer="21"/>
</package>
<package name="CHIP-LED0603" urn="urn:adsk.eagle:footprint:15671/1" library_version="4">
<description>&lt;b&gt;Hyper CHIPLED Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
LB Q993&lt;br&gt;
Source: http://www.osram.convergy.de/ ... Lb_q993.pdf</description>
<wire x1="-0.4" y1="0.45" x2="-0.4" y2="-0.45" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.45" x2="0.4" y2="-0.45" width="0.1016" layer="51"/>
<smd name="C" x="0" y="0.75" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0" y="-0.75" dx="0.8" dy="0.8" layer="1"/>
<text x="-0.635" y="-0.635" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-0.635" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.45" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="0.45" y2="-0.45" layer="51"/>
<rectangle x1="-0.45" y1="0" x2="-0.3" y2="0.3" layer="21"/>
<rectangle x1="0.3" y1="0" x2="0.45" y2="0.3" layer="21"/>
<rectangle x1="-0.15" y1="0" x2="0.15" y2="0.3" layer="21"/>
</package>
<package name="CHIP-LED0805" urn="urn:adsk.eagle:footprint:15672/1" library_version="4">
<description>&lt;b&gt;Hyper CHIPLED Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
LB R99A&lt;br&gt;
Source: http://www.osram.convergy.de/ ... lb_r99a.pdf</description>
<wire x1="-0.625" y1="0.45" x2="-0.625" y2="-0.45" width="0.1016" layer="51"/>
<wire x1="0.625" y1="0.45" x2="0.625" y2="-0.475" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.05" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="0" y="-1.05" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.675" y1="0" x2="-0.525" y2="0.3" layer="21"/>
<rectangle x1="0.525" y1="0" x2="0.675" y2="0.3" layer="21"/>
<rectangle x1="-0.15" y1="0" x2="0.15" y2="0.3" layer="21"/>
<rectangle x1="-0.675" y1="0.45" x2="0.675" y2="1.05" layer="51"/>
<rectangle x1="-0.675" y1="-1.05" x2="0.675" y2="-0.45" layer="51"/>
</package>
<package name="MINI-TOPLED-SANTANA" urn="urn:adsk.eagle:footprint:15673/1" library_version="4">
<description>&lt;b&gt;Mini TOPLED Santana®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG M470.pdf</description>
<wire x1="0.7" y1="-1" x2="0.35" y2="-1" width="0.1016" layer="21"/>
<wire x1="0.35" y1="-1" x2="-0.7" y2="-1" width="0.1016" layer="21"/>
<wire x1="-0.7" y1="-1" x2="-0.7" y2="1" width="0.1016" layer="21"/>
<wire x1="-0.7" y1="1" x2="0.7" y2="1" width="0.1016" layer="21"/>
<wire x1="0.7" y1="1" x2="0.7" y2="-0.65" width="0.1016" layer="21"/>
<wire x1="0.7" y1="-0.65" x2="0.7" y2="-1" width="0.1016" layer="21"/>
<wire x1="0.45" y1="-0.7" x2="-0.45" y2="-0.7" width="0.1016" layer="21"/>
<wire x1="-0.45" y1="-0.7" x2="-0.45" y2="0.7" width="0.1016" layer="21"/>
<wire x1="-0.45" y1="0.7" x2="0.45" y2="0.7" width="0.1016" layer="21"/>
<wire x1="0.45" y1="0.7" x2="0.45" y2="-0.7" width="0.1016" layer="21"/>
<wire x1="0.7" y1="-0.65" x2="0.35" y2="-1" width="0.1016" layer="21"/>
<smd name="C" x="0" y="-2.2" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.2" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<text x="-1.27" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.55" y1="1.5" x2="0.55" y2="2.1" layer="29"/>
<rectangle x1="-0.55" y1="-2.1" x2="0.55" y2="-1.5" layer="29"/>
<rectangle x1="-0.5" y1="-2.05" x2="0.5" y2="-1.55" layer="31"/>
<rectangle x1="-0.5" y1="1.55" x2="0.5" y2="2.05" layer="31"/>
<rectangle x1="-0.2" y1="-0.4" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.5" y1="-2.1" x2="0.5" y2="-1.4" layer="51"/>
<rectangle x1="-0.5" y1="1.4" x2="0.5" y2="2.05" layer="51"/>
<rectangle x1="-0.5" y1="1" x2="0.5" y2="1.4" layer="21"/>
<rectangle x1="-0.5" y1="-1.4" x2="0.5" y2="-1.05" layer="21"/>
<hole x="0" y="0" drill="2.7"/>
</package>
<package name="CHIPLED_0805" urn="urn:adsk.eagle:footprint:15674/1" library_version="4">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_R971.pdf</description>
<wire x1="-0.35" y1="0.925" x2="0.35" y2="0.925" width="0.1016" layer="51" curve="162.394521"/>
<wire x1="-0.35" y1="-0.925" x2="0.35" y2="-0.925" width="0.1016" layer="51" curve="-162.394521"/>
<wire x1="0.575" y1="0.525" x2="0.575" y2="-0.525" width="0.1016" layer="51"/>
<wire x1="-0.575" y1="-0.5" x2="-0.575" y2="0.925" width="0.1016" layer="51"/>
<circle x="-0.45" y="0.85" radius="0.103" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.05" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="0" y="-1.05" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="0.3" y1="0.5" x2="0.625" y2="1" layer="51"/>
<rectangle x1="-0.325" y1="0.5" x2="-0.175" y2="0.75" layer="51"/>
<rectangle x1="0.175" y1="0.5" x2="0.325" y2="0.75" layer="51"/>
<rectangle x1="-0.2" y1="0.5" x2="0.2" y2="0.675" layer="51"/>
<rectangle x1="0.3" y1="-1" x2="0.625" y2="-0.5" layer="51"/>
<rectangle x1="-0.625" y1="-1" x2="-0.3" y2="-0.5" layer="51"/>
<rectangle x1="0.175" y1="-0.75" x2="0.325" y2="-0.5" layer="51"/>
<rectangle x1="-0.325" y1="-0.75" x2="-0.175" y2="-0.5" layer="51"/>
<rectangle x1="-0.2" y1="-0.675" x2="0.2" y2="-0.5" layer="51"/>
<rectangle x1="-0.1" y1="0" x2="0.1" y2="0.2" layer="21"/>
<rectangle x1="-0.6" y1="0.5" x2="-0.3" y2="0.8" layer="51"/>
<rectangle x1="-0.625" y1="0.925" x2="-0.3" y2="1" layer="51"/>
</package>
<package name="CHIPLED_1206" urn="urn:adsk.eagle:footprint:15675/1" library_version="4">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_LY N971.pdf</description>
<wire x1="-0.4" y1="1.6" x2="0.4" y2="1.6" width="0.1016" layer="51" curve="172.619069"/>
<wire x1="-0.8" y1="-0.95" x2="-0.8" y2="0.95" width="0.1016" layer="51"/>
<wire x1="0.8" y1="0.95" x2="0.8" y2="-0.95" width="0.1016" layer="51"/>
<circle x="-0.55" y="1.425" radius="0.1" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.75" dx="1.5" dy="1.5" layer="1"/>
<smd name="A" x="0" y="-1.75" dx="1.5" dy="1.5" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.85" y1="1.525" x2="-0.35" y2="1.65" layer="51"/>
<rectangle x1="-0.85" y1="1.225" x2="-0.625" y2="1.55" layer="51"/>
<rectangle x1="-0.45" y1="1.225" x2="-0.325" y2="1.45" layer="51"/>
<rectangle x1="-0.65" y1="1.225" x2="-0.225" y2="1.35" layer="51"/>
<rectangle x1="0.35" y1="1.3" x2="0.85" y2="1.65" layer="51"/>
<rectangle x1="0.25" y1="1.225" x2="0.85" y2="1.35" layer="51"/>
<rectangle x1="-0.85" y1="0.95" x2="0.85" y2="1.25" layer="51"/>
<rectangle x1="-0.85" y1="-1.65" x2="0.85" y2="-0.95" layer="51"/>
<rectangle x1="-0.85" y1="0.35" x2="-0.525" y2="0.775" layer="21"/>
<rectangle x1="0.525" y1="0.35" x2="0.85" y2="0.775" layer="21"/>
<rectangle x1="-0.175" y1="0" x2="0.175" y2="0.35" layer="21"/>
</package>
<package name="CHIPLED_0603" urn="urn:adsk.eagle:footprint:15676/1" library_version="4">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_LY Q971.pdf</description>
<wire x1="-0.3" y1="0.8" x2="0.3" y2="0.8" width="0.1016" layer="51" curve="170.055574"/>
<wire x1="-0.275" y1="-0.825" x2="0.275" y2="-0.825" width="0.0508" layer="51" curve="-180"/>
<wire x1="-0.4" y1="0.375" x2="-0.4" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.35" x2="0.4" y2="-0.35" width="0.1016" layer="51"/>
<circle x="-0.35" y="0.625" radius="0.075" width="0.0508" layer="51"/>
<smd name="C" x="0" y="0.75" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0" y="-0.75" dx="0.8" dy="0.8" layer="1"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.7" x2="-0.25" y2="0.85" layer="51"/>
<rectangle x1="-0.275" y1="0.55" x2="-0.225" y2="0.6" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="-0.4" y2="0.725" layer="51"/>
<rectangle x1="0.25" y1="0.55" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="0.45" y2="0.575" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="-0.25" y2="-0.35" layer="51"/>
<rectangle x1="0.25" y1="-0.85" x2="0.45" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.575" x2="0.275" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.65" x2="-0.175" y2="-0.55" layer="51"/>
<rectangle x1="0.175" y1="-0.65" x2="0.275" y2="-0.55" layer="51"/>
<rectangle x1="-0.125" y1="0" x2="0.125" y2="0.25" layer="21"/>
</package>
<package name="CHIPLED-0603-TTW" urn="urn:adsk.eagle:footprint:15677/1" library_version="4">
<description>&lt;b&gt;CHIPLED-0603&lt;/b&gt;&lt;p&gt;
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603&lt;br&gt;
Package able to withstand TTW-soldering heat&lt;br&gt;
Package suitable for TTW-soldering&lt;br&gt;
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<wire x1="-0.3" y1="0.8" x2="0.3" y2="0.8" width="0.1016" layer="51" curve="170.055574"/>
<wire x1="-0.275" y1="-0.825" x2="0.275" y2="-0.825" width="0.0508" layer="51" curve="-180"/>
<wire x1="-0.4" y1="0.375" x2="-0.4" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.35" x2="0.4" y2="-0.35" width="0.1016" layer="51"/>
<circle x="-0.35" y="0.625" radius="0.075" width="0.0508" layer="51"/>
<smd name="C" x="0" y="0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="-0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A@1" x="0" y="-0.5" dx="0.35" dy="0.35" layer="1"/>
<smd name="C@1" x="0" y="0.5" dx="0.35" dy="0.35" layer="1"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.7" x2="-0.25" y2="0.85" layer="51"/>
<rectangle x1="-0.275" y1="0.55" x2="-0.225" y2="0.6" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="-0.4" y2="0.725" layer="51"/>
<rectangle x1="0.25" y1="0.55" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="0.45" y2="0.575" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="-0.25" y2="-0.35" layer="51"/>
<rectangle x1="0.25" y1="-0.85" x2="0.45" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.575" x2="0.275" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.65" x2="-0.175" y2="-0.55" layer="51"/>
<rectangle x1="0.175" y1="-0.65" x2="0.275" y2="-0.55" layer="51"/>
<rectangle x1="-0.125" y1="0" x2="0.125" y2="0.25" layer="21"/>
<rectangle x1="-0.175" y1="0.325" x2="0.175" y2="0.7" layer="29"/>
<rectangle x1="-0.4" y1="0.625" x2="0.4" y2="1.125" layer="29"/>
<rectangle x1="-0.4" y1="-1.125" x2="0.4" y2="-0.625" layer="29"/>
<rectangle x1="-0.175" y1="-0.675" x2="0.175" y2="-0.325" layer="29"/>
</package>
<package name="SMARTLED-TTW" urn="urn:adsk.eagle:footprint:15678/1" library_version="4">
<description>&lt;b&gt;SmartLED TTW&lt;/b&gt;&lt;p&gt;
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603&lt;br&gt;
Package able to withstand TTW-soldering heat&lt;br&gt;
Package suitable for TTW-soldering&lt;br&gt;
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<wire x1="-0.35" y1="0.6" x2="0.35" y2="0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.35" y1="0.6" x2="0.35" y2="-0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.6" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.15" y1="-0.6" x2="-0.35" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.35" y1="-0.6" x2="-0.35" y2="0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.4" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<smd name="C" x="0" y="0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="-0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A@1" x="0" y="-0.5" dx="0.35" dy="0.35" layer="1" stop="no" cream="no"/>
<smd name="C@1" x="0" y="0.5" dx="0.35" dy="0.35" layer="1" stop="no" cream="no"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.175" y1="0.325" x2="0.175" y2="0.7" layer="29"/>
<rectangle x1="-0.15" y1="-0.35" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.15" y1="0.6" x2="0.15" y2="0.85" layer="51"/>
<rectangle x1="-0.15" y1="-0.85" x2="0.15" y2="-0.6" layer="51"/>
<rectangle x1="-0.225" y1="0.3" x2="0.225" y2="0.975" layer="31"/>
<rectangle x1="-0.175" y1="-0.7" x2="0.175" y2="-0.325" layer="29" rot="R180"/>
<rectangle x1="-0.225" y1="-0.975" x2="0.225" y2="-0.3" layer="31" rot="R180"/>
</package>
<package name="LUMILED+" urn="urn:adsk.eagle:footprint:15679/1" library_version="4">
<description>&lt;b&gt;Lumileds Lighting. LUXEON®&lt;/b&gt; with cool pad&lt;p&gt;
Source: K2.pdf</description>
<wire x1="-3.575" y1="2.3375" x2="-2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="3.575" x2="2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="2.3375" x2="3.575" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="-3.575" x2="-2.3375" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="-3.575" x2="-2.5" y2="-3.4125" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="-3.4125" x2="-3.4125" y2="-2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="-3.4125" y1="-2.5" x2="-3.575" y2="-2.3375" width="0.2032" layer="21"/>
<wire x1="-3.575" y1="-2.3375" x2="-3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="2.3375" y1="3.575" x2="2.5" y2="3.4125" width="0.2032" layer="21"/>
<wire x1="2.5" y1="3.4125" x2="3.4125" y2="2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="3.4125" y1="2.5" x2="3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="-1.725" y1="2.225" x2="-1.0625" y2="2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<wire x1="1.725" y1="-2.225" x2="1.0625" y2="-2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<circle x="0" y="0" radius="2.725" width="0.2032" layer="51"/>
<smd name="1NC" x="-5.2" y="1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="2+" x="-5.2" y="-1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="3NC" x="5.2" y="-1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<smd name="4-" x="5.2" y="1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<text x="-3.175" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.975" y1="0.575" x2="-3.625" y2="1.6" layer="51"/>
<rectangle x1="-5.975" y1="-1.6" x2="-3.625" y2="-0.575" layer="51"/>
<rectangle x1="3.625" y1="-1.6" x2="5.975" y2="-0.575" layer="51" rot="R180"/>
<rectangle x1="3.625" y1="0.575" x2="5.975" y2="1.6" layer="51" rot="R180"/>
<polygon width="0.4064" layer="1">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="29">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="31">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
</package>
<package name="LUMILED" urn="urn:adsk.eagle:footprint:15680/1" library_version="4">
<description>&lt;b&gt;Lumileds Lighting. LUXEON®&lt;/b&gt; without cool pad&lt;p&gt;
Source: K2.pdf</description>
<wire x1="-3.575" y1="2.3375" x2="-2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="3.575" x2="2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="2.3375" x2="3.575" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="-3.575" x2="-2.3375" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="-3.575" x2="-2.5" y2="-3.4125" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="-3.4125" x2="-3.4125" y2="-2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="-3.4125" y1="-2.5" x2="-3.575" y2="-2.3375" width="0.2032" layer="21"/>
<wire x1="-3.575" y1="-2.3375" x2="-3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="2.3375" y1="3.575" x2="2.5" y2="3.4125" width="0.2032" layer="21"/>
<wire x1="2.5" y1="3.4125" x2="3.4125" y2="2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="3.4125" y1="2.5" x2="3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="-1.725" y1="2.225" x2="-1.0625" y2="2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<wire x1="1.725" y1="-2.225" x2="1.0625" y2="-2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<circle x="0" y="0" radius="2.725" width="0.2032" layer="51"/>
<smd name="1NC" x="-5.2" y="1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="2+" x="-5.2" y="-1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="3NC" x="5.2" y="-1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<smd name="4-" x="5.2" y="1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<text x="-3.175" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.975" y1="0.575" x2="-3.625" y2="1.6" layer="51"/>
<rectangle x1="-5.975" y1="-1.6" x2="-3.625" y2="-0.575" layer="51"/>
<rectangle x1="3.625" y1="-1.6" x2="5.975" y2="-0.575" layer="51" rot="R180"/>
<rectangle x1="3.625" y1="0.575" x2="5.975" y2="1.6" layer="51" rot="R180"/>
<polygon width="0.4064" layer="29">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="31">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
</package>
<package name="LED10MM" urn="urn:adsk.eagle:footprint:15681/1" library_version="4">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
10 mm, round</description>
<wire x1="5.08" y1="-2.54" x2="5.08" y2="2.54" width="0.254" layer="21" curve="-306.869898"/>
<wire x1="4.445" y1="0" x2="0" y2="-4.445" width="0.127" layer="21" curve="-90"/>
<wire x1="3.81" y1="0" x2="0" y2="-3.81" width="0.127" layer="21" curve="-90"/>
<wire x1="3.175" y1="0" x2="0" y2="-3.175" width="0.127" layer="21" curve="-90"/>
<wire x1="2.54" y1="0" x2="0" y2="-2.54" width="0.127" layer="21" curve="-90"/>
<wire x1="-4.445" y1="0" x2="0" y2="4.445" width="0.127" layer="21" curve="-90"/>
<wire x1="-3.81" y1="0" x2="0" y2="3.81" width="0.127" layer="21" curve="-90"/>
<wire x1="-3.175" y1="0" x2="0" y2="3.175" width="0.127" layer="21" curve="-90"/>
<wire x1="-2.54" y1="0" x2="0" y2="2.54" width="0.127" layer="21" curve="-90"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.254" layer="21"/>
<circle x="0" y="0" radius="5.08" width="0.127" layer="21"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.6764" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="6.35" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="6.35" y="-1.27" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="KA-3528ASYC" urn="urn:adsk.eagle:footprint:15682/1" library_version="4">
<description>&lt;b&gt;SURFACE MOUNT LED LAMP&lt;/b&gt; 3.5x2.8mm&lt;p&gt;
Source: http://www.kingbright.com/manager/upload/pdf/KA-3528ASYC(Ver1189474662.1)</description>
<wire x1="-1.55" y1="1.35" x2="1.55" y2="1.35" width="0.1016" layer="21"/>
<wire x1="1.55" y1="1.35" x2="1.55" y2="-1.35" width="0.1016" layer="51"/>
<wire x1="1.55" y1="-1.35" x2="-1.55" y2="-1.35" width="0.1016" layer="21"/>
<wire x1="-1.55" y1="-1.35" x2="-1.55" y2="1.35" width="0.1016" layer="51"/>
<wire x1="-0.65" y1="0.95" x2="0.65" y2="0.95" width="0.1016" layer="21" curve="-68.40813"/>
<wire x1="0.65" y1="-0.95" x2="-0.65" y2="-0.95" width="0.1016" layer="21" curve="-68.40813"/>
<circle x="0" y="0" radius="1.15" width="0.1016" layer="51"/>
<smd name="A" x="-1.55" y="0" dx="1.5" dy="2.2" layer="1"/>
<smd name="C" x="1.55" y="0" dx="1.5" dy="2.2" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.75" y1="0.6" x2="-1.6" y2="1.1" layer="51"/>
<rectangle x1="-1.75" y1="-1.1" x2="-1.6" y2="-0.6" layer="51"/>
<rectangle x1="1.6" y1="-1.1" x2="1.75" y2="-0.6" layer="51" rot="R180"/>
<rectangle x1="1.6" y1="0.6" x2="1.75" y2="1.1" layer="51" rot="R180"/>
<polygon width="0.1016" layer="51">
<vertex x="1.55" y="-1.35"/>
<vertex x="1.55" y="-0.625"/>
<vertex x="0.825" y="-1.35"/>
</polygon>
<polygon width="0.1016" layer="21">
<vertex x="1.55" y="-1.35"/>
<vertex x="1.55" y="-1.175"/>
<vertex x="1" y="-1.175"/>
<vertex x="0.825" y="-1.35"/>
</polygon>
</package>
<package name="SML0805" urn="urn:adsk.eagle:footprint:15683/1" library_version="4">
<description>&lt;b&gt;SML0805-2CW-TR (0805 PROFILE)&lt;/b&gt; COOL WHITE&lt;p&gt;
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0093.pdf</description>
<wire x1="-0.95" y1="-0.55" x2="0.95" y2="-0.55" width="0.1016" layer="51"/>
<wire x1="0.95" y1="-0.55" x2="0.95" y2="0.55" width="0.1016" layer="51"/>
<wire x1="0.95" y1="0.55" x2="-0.95" y2="0.55" width="0.1016" layer="51"/>
<wire x1="-0.95" y1="0.55" x2="-0.95" y2="-0.55" width="0.1016" layer="51"/>
<wire x1="-0.175" y1="-0.025" x2="0" y2="0.15" width="0.0634" layer="21"/>
<wire x1="0" y1="0.15" x2="0.15" y2="0" width="0.0634" layer="21"/>
<wire x1="0.15" y1="0" x2="-0.025" y2="-0.175" width="0.0634" layer="21"/>
<wire x1="-0.025" y1="-0.175" x2="-0.175" y2="-0.025" width="0.0634" layer="21"/>
<circle x="-0.275" y="0.4" radius="0.125" width="0" layer="21"/>
<smd name="C" x="-1.05" y="0" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="1.05" y="0" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.5" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.5" y="-2" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="SML1206" urn="urn:adsk.eagle:footprint:15684/1" library_version="4">
<description>&lt;b&gt;SML10XXKH-TR (HIGH INTENSITY) LED&lt;/b&gt;&lt;p&gt;
&lt;table&gt;
&lt;tr&gt;&lt;td&gt;SML10R3KH-TR&lt;/td&gt;&lt;td&gt;ULTRA RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10E3KH-TR&lt;/td&gt;&lt;td&gt;SUPER REDSUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10O3KH-TR&lt;/td&gt;&lt;td&gt;SUPER ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10PY3KH-TR&lt;/td&gt;&lt;td&gt;PURE YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10OY3KH-TR&lt;/td&gt;&lt;td&gt;ULTRA YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10AG3KH-TR&lt;/td&gt;&lt;td&gt;AQUA GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10BG3KH-TR&lt;/td&gt;&lt;td&gt;BLUE GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10PB1KH-TR&lt;/td&gt;&lt;td&gt;SUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10CW1KH-TR&lt;/td&gt;&lt;td&gt;WHITE&lt;/td&gt;&lt;/tr&gt;
&lt;/table&gt;

Source: http://www.ledtronics.com/ds/smd-1206/dstr0094.PDF</description>
<wire x1="-1.5" y1="0.5" x2="-1.5" y2="-0.5" width="0.2032" layer="51" curve="-180"/>
<wire x1="1.5" y1="-0.5" x2="1.5" y2="0.5" width="0.2032" layer="51" curve="-180"/>
<wire x1="-1.55" y1="0.75" x2="1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.55" y1="-0.75" x2="-1.55" y2="-0.75" width="0.1016" layer="51"/>
<circle x="-0.725" y="0.525" radius="0.125" width="0" layer="21"/>
<smd name="C" x="-1.75" y="0" dx="1.5" dy="1.5" layer="1"/>
<smd name="A" x="1.75" y="0" dx="1.5" dy="1.5" layer="1"/>
<text x="-1.5" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.5" y="-2.5" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="0.4" x2="-1.15" y2="0.8" layer="51"/>
<rectangle x1="-1.6" y1="-0.8" x2="-1.15" y2="-0.4" layer="51"/>
<rectangle x1="-1.175" y1="-0.6" x2="-1" y2="-0.275" layer="51"/>
<rectangle x1="1.15" y1="-0.8" x2="1.6" y2="-0.4" layer="51" rot="R180"/>
<rectangle x1="1.15" y1="0.4" x2="1.6" y2="0.8" layer="51" rot="R180"/>
<rectangle x1="1" y1="0.275" x2="1.175" y2="0.6" layer="51" rot="R180"/>
<rectangle x1="-0.1" y1="-0.1" x2="0.1" y2="0.1" layer="21"/>
</package>
<package name="SML0603" urn="urn:adsk.eagle:footprint:15685/1" library_version="4">
<description>&lt;b&gt;SML0603-XXX (HIGH INTENSITY) LED&lt;/b&gt;&lt;p&gt;
&lt;table&gt;
&lt;tr&gt;&lt;td&gt;AG3K&lt;/td&gt;&lt;td&gt;AQUA GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;B1K&lt;/td&gt;&lt;td&gt;SUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;R1K&lt;/td&gt;&lt;td&gt;SUPER RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;R3K&lt;/td&gt;&lt;td&gt;ULTRA RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;O3K&lt;/td&gt;&lt;td&gt;SUPER ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;O3KH&lt;/td&gt;&lt;td&gt;SOFT ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;Y3KH&lt;/td&gt;&lt;td&gt;SUPER YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;Y3K&lt;/td&gt;&lt;td&gt;SUPER YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;2CW&lt;/td&gt;&lt;td&gt;WHITE&lt;/td&gt;&lt;/tr&gt;
&lt;/table&gt;
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0092.pdf</description>
<wire x1="-0.75" y1="0.35" x2="0.75" y2="0.35" width="0.1016" layer="51"/>
<wire x1="0.75" y1="0.35" x2="0.75" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.75" y1="-0.35" x2="-0.75" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="-0.75" y1="-0.35" x2="-0.75" y2="0.35" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="0.3" x2="-0.45" y2="-0.3" width="0.1016" layer="51"/>
<wire x1="0.45" y1="0.3" x2="0.45" y2="-0.3" width="0.1016" layer="51"/>
<wire x1="-0.2" y1="0.35" x2="0.2" y2="0.35" width="0.1016" layer="21"/>
<wire x1="-0.2" y1="-0.35" x2="0.2" y2="-0.35" width="0.1016" layer="21"/>
<smd name="C" x="-0.75" y="0" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0.75" y="0" dx="0.8" dy="0.8" layer="1"/>
<text x="-1" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1" y="-2" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.4" y1="0.175" x2="0" y2="0.4" layer="51"/>
<rectangle x1="-0.25" y1="0.175" x2="0" y2="0.4" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="1206" urn="urn:adsk.eagle:package:15796/2" type="model" library_version="4">
<description>CHICAGO MINIATURE LAMP, INC.
7022X Series SMT LEDs 1206 Package Size</description>
<packageinstances>
<packageinstance name="1206"/>
</packageinstances>
</package3d>
<package3d name="LD260" urn="urn:adsk.eagle:package:15794/1" type="box" library_version="4">
<description>LED
5 mm, square, Siemens</description>
<packageinstances>
<packageinstance name="LD260"/>
</packageinstances>
</package3d>
<package3d name="LED2X5" urn="urn:adsk.eagle:package:15800/1" type="box" library_version="4">
<description>LED
2 x 5 mm, rectangle</description>
<packageinstances>
<packageinstance name="LED2X5"/>
</packageinstances>
</package3d>
<package3d name="LED3MM" urn="urn:adsk.eagle:package:15797/1" type="box" library_version="4">
<description>LED
3 mm, round</description>
<packageinstances>
<packageinstance name="LED3MM"/>
</packageinstances>
</package3d>
<package3d name="LED5MM" urn="urn:adsk.eagle:package:15799/2" type="model" library_version="4">
<description>LED
5 mm, round</description>
<packageinstances>
<packageinstance name="LED5MM"/>
</packageinstances>
</package3d>
<package3d name="LSU260" urn="urn:adsk.eagle:package:15805/1" type="box" library_version="4">
<description>LED
1 mm, round, Siemens</description>
<packageinstances>
<packageinstance name="LSU260"/>
</packageinstances>
</package3d>
<package3d name="LZR181" urn="urn:adsk.eagle:package:15808/1" type="box" library_version="4">
<description>LED BLOCK
1 LED, Siemens</description>
<packageinstances>
<packageinstance name="LZR181"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B152" urn="urn:adsk.eagle:package:15803/1" type="box" library_version="4">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B152"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B153" urn="urn:adsk.eagle:package:15804/1" type="box" library_version="4">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B153"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B155" urn="urn:adsk.eagle:package:15807/1" type="box" library_version="4">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B155"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B156" urn="urn:adsk.eagle:package:15806/1" type="box" library_version="4">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B156"/>
</packageinstances>
</package3d>
<package3d name="SFH480" urn="urn:adsk.eagle:package:15809/1" type="box" library_version="4">
<description>IR LED
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking
Inifineon</description>
<packageinstances>
<packageinstance name="SFH480"/>
</packageinstances>
</package3d>
<package3d name="SFH482" urn="urn:adsk.eagle:package:15795/1" type="box" library_version="4">
<description>IR LED
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking
Inifineon</description>
<packageinstances>
<packageinstance name="SFH482"/>
</packageinstances>
</package3d>
<package3d name="U57X32" urn="urn:adsk.eagle:package:15789/1" type="box" library_version="4">
<description>LED
rectangle, 5.7 x 3.2 mm</description>
<packageinstances>
<packageinstance name="U57X32"/>
</packageinstances>
</package3d>
<package3d name="IRL80A" urn="urn:adsk.eagle:package:15810/1" type="box" library_version="4">
<description>IR LED
IR transmitter Siemens</description>
<packageinstances>
<packageinstance name="IRL80A"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-2" urn="urn:adsk.eagle:package:15817/1" type="box" library_version="4">
<description>TOPLED® High-optical Power LED (HOP)
Source: http://www.osram.convergy.de/ ... ls_t675.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-2"/>
</packageinstances>
</package3d>
<package3d name="OSRAM-MINI-TOP-LED" urn="urn:adsk.eagle:package:15811/1" type="box" library_version="4">
<description>BLUE LINETM Hyper Mini TOPLED® Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LB M676.pdf</description>
<packageinstances>
<packageinstance name="OSRAM-MINI-TOP-LED"/>
</packageinstances>
</package3d>
<package3d name="OSRAM-SIDELED" urn="urn:adsk.eagle:package:15812/1" type="box" library_version="4">
<description>Super SIDELED® High-Current LED
LG A672, LP A672 
Source: http://www.osram.convergy.de/ ... LG_LP_A672.pdf (2004.05.13)</description>
<packageinstances>
<packageinstance name="OSRAM-SIDELED"/>
</packageinstances>
</package3d>
<package3d name="SMART-LED" urn="urn:adsk.eagle:package:15814/1" type="box" library_version="4">
<description>SmartLEDTM Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY L896.pdf</description>
<packageinstances>
<packageinstance name="SMART-LED"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-2-TOPLED-RG" urn="urn:adsk.eagle:package:15813/1" type="box" library_version="4">
<description>Hyper TOPLED® RG Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY T776.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-2-TOPLED-RG"/>
</packageinstances>
</package3d>
<package3d name="MICRO-SIDELED" urn="urn:adsk.eagle:package:15815/1" type="box" library_version="4">
<description>Hyper Micro SIDELED®
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY Y876.pdf</description>
<packageinstances>
<packageinstance name="MICRO-SIDELED"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-4" urn="urn:adsk.eagle:package:15816/1" type="box" library_version="4">
<description>Power TOPLED®
Source: http://www.osram.convergy.de/ ... LA_LO_LA_LY E67B.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-4"/>
</packageinstances>
</package3d>
<package3d name="CHIP-LED0603" urn="urn:adsk.eagle:package:15819/3" type="model" library_version="4">
<description>Hyper CHIPLED Hyper-Bright LED
LB Q993
Source: http://www.osram.convergy.de/ ... Lb_q993.pdf</description>
<packageinstances>
<packageinstance name="CHIP-LED0603"/>
</packageinstances>
</package3d>
<package3d name="CHIP-LED0805" urn="urn:adsk.eagle:package:15818/2" type="model" library_version="4">
<description>Hyper CHIPLED Hyper-Bright LED
LB R99A
Source: http://www.osram.convergy.de/ ... lb_r99a.pdf</description>
<packageinstances>
<packageinstance name="CHIP-LED0805"/>
</packageinstances>
</package3d>
<package3d name="MINI-TOPLED-SANTANA" urn="urn:adsk.eagle:package:15820/1" type="box" library_version="4">
<description>Mini TOPLED Santana®
Source: http://www.osram.convergy.de/ ... LG M470.pdf</description>
<packageinstances>
<packageinstance name="MINI-TOPLED-SANTANA"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_0805" urn="urn:adsk.eagle:package:15821/2" type="model" library_version="4">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_R971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_0805"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_1206" urn="urn:adsk.eagle:package:15823/2" type="model" library_version="4">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_LY N971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_1206"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_0603" urn="urn:adsk.eagle:package:15822/2" type="model" library_version="4">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_LY Q971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_0603"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED-0603-TTW" urn="urn:adsk.eagle:package:15824/1" type="box" library_version="4">
<description>CHIPLED-0603
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603
Package able to withstand TTW-soldering heat
Package suitable for TTW-soldering
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED-0603-TTW"/>
</packageinstances>
</package3d>
<package3d name="SMARTLED-TTW" urn="urn:adsk.eagle:package:15825/1" type="box" library_version="4">
<description>SmartLED TTW
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603
Package able to withstand TTW-soldering heat
Package suitable for TTW-soldering
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<packageinstances>
<packageinstance name="SMARTLED-TTW"/>
</packageinstances>
</package3d>
<package3d name="LUMILED+" urn="urn:adsk.eagle:package:15826/1" type="box" library_version="4">
<description>Lumileds Lighting. LUXEON® with cool pad
Source: K2.pdf</description>
<packageinstances>
<packageinstance name="LUMILED+"/>
</packageinstances>
</package3d>
<package3d name="LUMILED" urn="urn:adsk.eagle:package:15827/1" type="box" library_version="4">
<description>Lumileds Lighting. LUXEON® without cool pad
Source: K2.pdf</description>
<packageinstances>
<packageinstance name="LUMILED"/>
</packageinstances>
</package3d>
<package3d name="LED10MM" urn="urn:adsk.eagle:package:15828/1" type="box" library_version="4">
<description>LED
10 mm, round</description>
<packageinstances>
<packageinstance name="LED10MM"/>
</packageinstances>
</package3d>
<package3d name="KA-3528ASYC" urn="urn:adsk.eagle:package:15831/1" type="box" library_version="4">
<description>SURFACE MOUNT LED LAMP 3.5x2.8mm
Source: http://www.kingbright.com/manager/upload/pdf/KA-3528ASYC(Ver1189474662.1)</description>
<packageinstances>
<packageinstance name="KA-3528ASYC"/>
</packageinstances>
</package3d>
<package3d name="SML0805" urn="urn:adsk.eagle:package:15830/1" type="box" library_version="4">
<description>SML0805-2CW-TR (0805 PROFILE) COOL WHITE
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0093.pdf</description>
<packageinstances>
<packageinstance name="SML0805"/>
</packageinstances>
</package3d>
<package3d name="SML1206" urn="urn:adsk.eagle:package:15829/1" type="box" library_version="4">
<description>SML10XXKH-TR (HIGH INTENSITY) LED

SML10R3KH-TRULTRA RED
SML10E3KH-TRSUPER REDSUPER BLUE
SML10O3KH-TRSUPER ORANGE
SML10PY3KH-TRPURE YELLOW
SML10OY3KH-TRULTRA YELLOW
SML10AG3KH-TRAQUA GREEN
SML10BG3KH-TRBLUE GREEN
SML10PB1KH-TRSUPER BLUE
SML10CW1KH-TRWHITE


Source: http://www.ledtronics.com/ds/smd-1206/dstr0094.PDF</description>
<packageinstances>
<packageinstance name="SML1206"/>
</packageinstances>
</package3d>
<package3d name="SML0603" urn="urn:adsk.eagle:package:15832/1" type="box" library_version="4">
<description>SML0603-XXX (HIGH INTENSITY) LED

AG3KAQUA GREEN
B1KSUPER BLUE
R1KSUPER RED
R3KULTRA RED
O3KSUPER ORANGE
O3KHSOFT ORANGE
Y3KHSUPER YELLOW
Y3KSUPER YELLOW
2CWWHITE

Source: http://www.ledtronics.com/ds/smd-0603/Dstr0092.pdf</description>
<packageinstances>
<packageinstance name="SML0603"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="LED" urn="urn:adsk.eagle:symbol:15639/2" library_version="4">
<wire x1="1.27" y1="0" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-2.032" y1="-0.762" x2="-3.429" y2="-2.159" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="-3.302" y2="-3.302" width="0.1524" layer="94"/>
<text x="3.556" y="-4.572" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="5.715" y="-4.572" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="C" x="0" y="-5.08" visible="off" length="short" direction="pas" rot="R90"/>
<pin name="A" x="0" y="2.54" visible="off" length="short" direction="pas" rot="R270"/>
<polygon width="0.1524" layer="94">
<vertex x="-3.429" y="-2.159"/>
<vertex x="-3.048" y="-1.27"/>
<vertex x="-2.54" y="-1.778"/>
</polygon>
<polygon width="0.1524" layer="94">
<vertex x="-3.302" y="-3.302"/>
<vertex x="-2.921" y="-2.413"/>
<vertex x="-2.413" y="-2.921"/>
</polygon>
</symbol>
</symbols>
<devicesets>
<deviceset name="LED" urn="urn:adsk.eagle:component:15916/10" prefix="LED" uservalue="yes" library_version="5">
<description>&lt;b&gt;LED&lt;/b&gt;&lt;p&gt;
&lt;u&gt;OSRAM&lt;/u&gt;:&lt;br&gt;

- &lt;u&gt;CHIPLED&lt;/u&gt;&lt;br&gt;
LG R971, LG N971, LY N971, LG Q971, LY Q971, LO R971, LY R971
LH N974, LH R974&lt;br&gt;
LS Q976, LO Q976, LY Q976&lt;br&gt;
LO Q996&lt;br&gt;

- &lt;u&gt;Hyper CHIPLED&lt;/u&gt;&lt;br&gt;
LW Q18S&lt;br&gt;
LB Q993, LB Q99A, LB R99A&lt;br&gt;

- &lt;u&gt;SideLED&lt;/u&gt;&lt;br&gt;
LS A670, LO A670, LY A670, LG A670, LP A670&lt;br&gt;
LB A673, LV A673, LT A673, LW A673&lt;br&gt;
LH A674&lt;br&gt;
LY A675&lt;br&gt;
LS A676, LA A676, LO A676, LY A676, LW A676&lt;br&gt;
LS A679, LY A679, LG A679&lt;br&gt;

-  &lt;u&gt;Hyper Micro SIDELED®&lt;/u&gt;&lt;br&gt;
LS Y876, LA Y876, LO Y876, LY Y876&lt;br&gt;
LT Y87S&lt;br&gt;

- &lt;u&gt;SmartLED&lt;/u&gt;&lt;br&gt;
LW L88C, LW L88S&lt;br&gt;
LB L89C, LB L89S, LG L890&lt;br&gt;
LS L89K, LO L89K, LY L89K&lt;br&gt;
LS L896, LA L896, LO L896, LY L896&lt;br&gt;

- &lt;u&gt;TOPLED&lt;/u&gt;&lt;br&gt;
LS T670, LO T670, LY T670, LG T670, LP T670&lt;br&gt;
LSG T670, LSP T670, LSY T670, LOP T670, LYG T670&lt;br&gt;
LG T671, LOG T671, LSG T671&lt;br&gt;
LB T673, LV T673, LT T673, LW T673&lt;br&gt;
LH T674&lt;br&gt;
LS T676, LA T676, LO T676, LY T676, LB T676, LH T676, LSB T676, LW T676&lt;br&gt;
LB T67C, LV T67C, LT T67C, LS T67K, LO T67K, LY T67K, LW E67C&lt;br&gt;
LS E67B, LA E67B, LO E67B, LY E67B, LB E67C, LV E67C, LT E67C&lt;br&gt;
LW T67C&lt;br&gt;
LS T679, LY T679, LG T679&lt;br&gt;
LS T770, LO T770, LY T770, LG T770, LP T770&lt;br&gt;
LB T773, LV T773, LT T773, LW T773&lt;br&gt;
LH T774&lt;br&gt;
LS E675, LA E675, LY E675, LS T675&lt;br&gt;
LS T776, LA T776, LO T776, LY T776, LB T776&lt;br&gt;
LHGB T686&lt;br&gt;
LT T68C, LB T68C&lt;br&gt;

- &lt;u&gt;Hyper Mini TOPLED®&lt;/u&gt;&lt;br&gt;
LB M676&lt;br&gt;

- &lt;u&gt;Mini TOPLED Santana®&lt;/u&gt;&lt;br&gt;
LG M470&lt;br&gt;
LS M47K, LO M47K, LY M47K
&lt;p&gt;
Source: http://www.osram.convergy.de&lt;p&gt;

&lt;u&gt;LUXEON:&lt;/u&gt;&lt;br&gt;
- &lt;u&gt;LUMILED®&lt;/u&gt;&lt;br&gt;
LXK2-PW12-R00, LXK2-PW12-S00, LXK2-PW14-U00, LXK2-PW14-V00&lt;br&gt;
LXK2-PM12-R00, LXK2-PM12-S00, LXK2-PM14-U00&lt;br&gt;
LXK2-PE12-Q00, LXK2-PE12-R00, LXK2-PE12-S00, LXK2-PE14-T00, LXK2-PE14-U00&lt;br&gt;
LXK2-PB12-K00, LXK2-PB12-L00, LXK2-PB12-M00, LXK2-PB14-N00, LXK2-PB14-P00, LXK2-PB14-Q00&lt;br&gt;
LXK2-PR12-L00, LXK2-PR12-M00, LXK2-PR14-Q00, LXK2-PR14-R00&lt;br&gt;
LXK2-PD12-Q00, LXK2-PD12-R00, LXK2-PD12-S00&lt;br&gt;
LXK2-PH12-R00, LXK2-PH12-S00&lt;br&gt;
LXK2-PL12-P00, LXK2-PL12-Q00, LXK2-PL12-R00
&lt;p&gt;
Source: www.luxeon.com&lt;p&gt;

&lt;u&gt;KINGBRIGHT:&lt;/U&gt;&lt;p&gt;
KA-3528ASYC&lt;br&gt;
Source: www.kingbright.com</description>
<gates>
<gate name="G$1" symbol="LED" x="0" y="0"/>
</gates>
<devices>
<device name="SMT1206" package="1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15796/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="20" constant="no"/>
</technology>
</technologies>
</device>
<device name="LD260" package="LD260">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15794/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="SQR2X5" package="LED2X5">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15800/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
<device name="3MM" package="LED3MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15797/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="97" constant="no"/>
</technology>
</technologies>
</device>
<device name="5MM" package="LED5MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15799/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="93" constant="no"/>
</technology>
</technologies>
</device>
<device name="LSU260" package="LSU260">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15805/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="LZR181" package="LZR181">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15808/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
</technology>
</technologies>
</device>
<device name="B152" package="Q62902-B152">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15803/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="B153" package="Q62902-B153">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15804/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="B155" package="Q62902-B155">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15807/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="B156" package="Q62902-B156">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15806/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="SFH480" package="SFH480">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15809/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
<device name="SFH482" package="SFH482">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15795/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="SQR5.7X3.2" package="U57X32">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15789/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="IRL80A" package="IRL80A">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15810/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="P-LCC-2" package="P-LCC-2">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15817/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
<device name="MINI-TOP" package="OSRAM-MINI-TOP-LED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15811/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="SIDELED" package="OSRAM-SIDELED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15812/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
<device name="SMART-LED" package="SMART-LED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="B"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15814/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="P-LCC-2-BACK" package="P-LCC-2-TOPLED-RG">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15813/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="MICRO-SIDELED" package="MICRO-SIDELED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15815/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="P-LCC-4" package="P-LCC-4">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C@4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15816/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-LED0603" package="CHIP-LED0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15819/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="26" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIP-LED0805" package="CHIP-LED0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15818/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="68" constant="no"/>
</technology>
</technologies>
</device>
<device name="TOPLED-SANTANA" package="MINI-TOPLED-SANTANA">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15820/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIPLED_0805" package="CHIPLED_0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15821/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="52" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIPLED_1206" package="CHIPLED_1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15823/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="28" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIPLED_0603" package="CHIPLED_0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15822/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="28" constant="no"/>
</technology>
</technologies>
</device>
<device name="CHIPLED-0603-TTW" package="CHIPLED-0603-TTW">
<connects>
<connect gate="G$1" pin="A" pad="A@1"/>
<connect gate="G$1" pin="C" pad="C@1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15824/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="" package="SMARTLED-TTW">
<connects>
<connect gate="G$1" pin="A" pad="A@1"/>
<connect gate="G$1" pin="C" pad="C@1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15825/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
</technology>
</technologies>
</device>
<device name="-LUMILED+" package="LUMILED+">
<connects>
<connect gate="G$1" pin="A" pad="2+"/>
<connect gate="G$1" pin="C" pad="4-"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15826/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="-LUMILED" package="LUMILED">
<connects>
<connect gate="G$1" pin="A" pad="2+"/>
<connect gate="G$1" pin="C" pad="4-"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15827/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="10MM" package="LED10MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15828/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
</technology>
</technologies>
</device>
<device name="KA-3528ASYC" package="KA-3528ASYC">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15831/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="SML0805" package="SML0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15830/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
</technology>
</technologies>
</device>
<device name="SML1206" package="SML1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15829/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="6" constant="no"/>
</technology>
</technologies>
</device>
<device name="SML0603" package="SML0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15832/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="frames" urn="urn:adsk.eagle:library:229">
<description>&lt;b&gt;Frames for Sheet and Layout&lt;/b&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="A4L-LOC" urn="urn:adsk.eagle:symbol:13874/1" library_version="1">
<wire x1="256.54" y1="3.81" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="256.54" y1="8.89" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="256.54" y1="13.97" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="256.54" y1="19.05" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="3.81" x2="161.29" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="24.13" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<wire x1="215.265" y1="24.13" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="246.38" y1="3.81" x2="246.38" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="215.265" y2="8.89" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="3.81" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="215.265" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<text x="217.17" y="15.24" size="2.54" layer="94">&gt;DRAWING_NAME</text>
<text x="217.17" y="10.16" size="2.286" layer="94">&gt;LAST_DATE_TIME</text>
<text x="230.505" y="5.08" size="2.54" layer="94">&gt;SHEET</text>
<text x="216.916" y="4.953" size="2.54" layer="94">Sheet:</text>
<frame x1="0" y1="0" x2="260.35" y2="179.07" columns="6" rows="4" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="A4L-LOC" urn="urn:adsk.eagle:component:13926/1" prefix="FRAME" uservalue="yes" library_version="1">
<description>&lt;b&gt;FRAME&lt;/b&gt;&lt;p&gt;
DIN A4, landscape with location and doc. field</description>
<gates>
<gate name="G$1" symbol="A4L-LOC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="special" urn="urn:adsk.eagle:library:367">
<description>&lt;b&gt;Special Devices&lt;/b&gt;&lt;p&gt;
7-segment displays, switches, heatsinks, crystals, transformers, etc.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="EDG-03" urn="urn:adsk.eagle:footprint:26563/1" library_version="2">
<description>&lt;b&gt;DIP SWITCH&lt;/b&gt;</description>
<wire x1="-4.572" y1="-4.953" x2="4.572" y2="-4.953" width="0.1524" layer="21"/>
<wire x1="4.572" y1="4.953" x2="-4.572" y2="4.953" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="4.953" x2="-4.572" y2="1.651" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-1.651" x2="-4.064" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.651" x2="-4.064" y2="1.651" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.651" x2="-4.572" y2="1.651" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-1.651" x2="-4.572" y2="-4.953" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="1.651" x2="-4.572" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="4.572" y1="-4.953" x2="4.572" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="4.572" y1="1.651" x2="4.064" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.651" x2="4.064" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.651" x2="4.572" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="4.572" y1="1.651" x2="4.572" y2="4.953" width="0.1524" layer="21"/>
<wire x1="4.572" y1="-1.651" x2="4.572" y2="1.651" width="0.1524" layer="21"/>
<wire x1="1.778" y1="1.905" x2="1.778" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.778" y1="1.905" x2="3.302" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.905" x2="3.302" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.905" x2="1.778" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="1.905" x2="-0.762" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="1.905" x2="0.762" y2="1.905" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-1.905" x2="0.762" y2="1.905" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-1.905" x2="-0.762" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.905" x2="-3.302" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.905" x2="-1.778" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-1.905" x2="-1.778" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-1.905" x2="-3.302" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="0" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="-2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="0" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.794" y="-4.191" size="1.524" layer="51" ratio="10">1</text>
<text x="-3.81" y="2.54" size="1.524" layer="51" ratio="10">ON</text>
<text x="-0.508" y="-4.191" size="1.524" layer="51" ratio="10">2</text>
<text x="2.032" y="-4.191" size="1.524" layer="51" ratio="10">3</text>
<text x="-4.572" y="-6.604" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.572" y="5.334" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<rectangle x1="2.032" y1="-0.381" x2="3.048" y2="0" layer="21"/>
<rectangle x1="2.032" y1="-1.016" x2="3.048" y2="-0.635" layer="21"/>
<rectangle x1="2.032" y1="-1.651" x2="3.048" y2="-1.27" layer="21"/>
<rectangle x1="-0.508" y1="-0.381" x2="0.508" y2="0" layer="21"/>
<rectangle x1="-0.508" y1="-1.016" x2="0.508" y2="-0.635" layer="21"/>
<rectangle x1="-0.508" y1="-1.651" x2="0.508" y2="-1.27" layer="21"/>
<rectangle x1="-3.048" y1="-0.381" x2="-2.032" y2="0" layer="21"/>
<rectangle x1="-3.048" y1="-1.016" x2="-2.032" y2="-0.635" layer="21"/>
<rectangle x1="-3.048" y1="-1.651" x2="-2.032" y2="-1.27" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="EDG-03" urn="urn:adsk.eagle:package:26663/2" type="model" library_version="2">
<description>DIP SWITCH</description>
<packageinstances>
<packageinstance name="EDG-03"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="SW_DIP-3" urn="urn:adsk.eagle:symbol:26562/1" library_version="2">
<wire x1="-3.302" y1="3.048" x2="0" y2="3.048" width="0.1524" layer="94"/>
<wire x1="3.302" y1="3.048" x2="3.302" y2="2.032" width="0.1524" layer="94"/>
<wire x1="3.302" y1="2.032" x2="0" y2="2.032" width="0.1524" layer="94"/>
<wire x1="-3.302" y1="2.032" x2="-3.302" y2="3.048" width="0.1524" layer="94"/>
<wire x1="0" y1="3.048" x2="0" y2="2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="3.048" x2="3.302" y2="3.048" width="0.1524" layer="94"/>
<wire x1="0" y1="2.032" x2="-3.302" y2="2.032" width="0.1524" layer="94"/>
<wire x1="-5.08" y1="5.08" x2="5.08" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.302" y1="0.508" x2="0" y2="0.508" width="0.1524" layer="94"/>
<wire x1="3.302" y1="0.508" x2="3.302" y2="-0.508" width="0.1524" layer="94"/>
<wire x1="3.302" y1="-0.508" x2="0" y2="-0.508" width="0.1524" layer="94"/>
<wire x1="-3.302" y1="-0.508" x2="-3.302" y2="0.508" width="0.1524" layer="94"/>
<wire x1="0" y1="0.508" x2="0" y2="-0.508" width="0.1524" layer="94"/>
<wire x1="0" y1="0.508" x2="3.302" y2="0.508" width="0.1524" layer="94"/>
<wire x1="0" y1="-0.508" x2="-3.302" y2="-0.508" width="0.1524" layer="94"/>
<wire x1="-3.302" y1="-2.032" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="3.302" y1="-2.032" x2="3.302" y2="-3.048" width="0.1524" layer="94"/>
<wire x1="3.302" y1="-3.048" x2="0" y2="-3.048" width="0.1524" layer="94"/>
<wire x1="-3.302" y1="-3.048" x2="-3.302" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.032" x2="0" y2="-3.048" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.032" x2="3.302" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="-3.048" x2="-3.302" y2="-3.048" width="0.1524" layer="94"/>
<wire x1="-5.08" y1="5.08" x2="-5.08" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-5.08" y1="-5.08" x2="5.08" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="5.08" y1="-5.08" x2="5.08" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.397" y1="2.794" x2="1.905" y2="2.794" width="0" layer="94"/>
<wire x1="1.397" y1="2.286" x2="1.905" y2="2.286" width="0" layer="94"/>
<wire x1="1.397" y1="0.254" x2="1.905" y2="0.254" width="0" layer="94"/>
<wire x1="1.397" y1="-0.254" x2="1.905" y2="-0.254" width="0" layer="94"/>
<wire x1="1.397" y1="-2.286" x2="1.905" y2="-2.286" width="0" layer="94"/>
<wire x1="1.397" y1="-2.794" x2="1.905" y2="-2.794" width="0" layer="94"/>
<text x="-5.08" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="4.826" y="-2.667" size="1.27" layer="94" ratio="10" rot="R90">1</text>
<text x="4.826" y="-0.381" size="1.27" layer="94" ratio="10" rot="R90">2</text>
<text x="4.826" y="2.159" size="1.27" layer="94" ratio="10" rot="R90">3</text>
<text x="-3.556" y="-4.699" size="1.27" layer="94" ratio="10" rot="R90">ON</text>
<rectangle x1="0.381" y1="2.286" x2="1.397" y2="2.794" layer="94"/>
<rectangle x1="1.905" y1="2.286" x2="2.921" y2="2.794" layer="94"/>
<rectangle x1="0.381" y1="-0.254" x2="1.397" y2="0.254" layer="94"/>
<rectangle x1="1.905" y1="-0.254" x2="2.921" y2="0.254" layer="94"/>
<rectangle x1="0.381" y1="-2.794" x2="1.397" y2="-2.286" layer="94"/>
<rectangle x1="1.905" y1="-2.794" x2="2.921" y2="-2.286" layer="94"/>
<pin name="4" x="-7.62" y="2.54" visible="pad" length="short" direction="pas" swaplevel="3"/>
<pin name="5" x="-7.62" y="0" visible="pad" length="short" direction="pas" swaplevel="2"/>
<pin name="6" x="-7.62" y="-2.54" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="0" visible="pad" length="short" direction="pas" swaplevel="2" rot="R180"/>
<pin name="3" x="7.62" y="2.54" visible="pad" length="short" direction="pas" swaplevel="3" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SW_DIP-3" urn="urn:adsk.eagle:component:26696/3" prefix="SW" uservalue="yes" library_version="3">
<description>&lt;b&gt;DIP SWITCH&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="SW_DIP-3" x="0" y="0"/>
</gates>
<devices>
<device name="" package="EDG-03">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26663/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="wirepad" urn="urn:adsk.eagle:library:412">
<description>&lt;b&gt;Single Pads&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="2,54/1,0" urn="urn:adsk.eagle:footprint:30810/1" library_version="2">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="1" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="2,54/1,0" urn="urn:adsk.eagle:package:30828/1" type="box" library_version="2">
<description>THROUGH-HOLE PAD</description>
<packageinstances>
<packageinstance name="2,54/1,0"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="PAD" urn="urn:adsk.eagle:symbol:30808/1" library_version="2">
<wire x1="-1.016" y1="1.016" x2="1.016" y2="-1.016" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<text x="-1.143" y="1.8542" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.143" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="P" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="2,54/1,0" urn="urn:adsk.eagle:component:30846/2" prefix="PAD" uservalue="yes" library_version="2">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<gates>
<gate name="P" symbol="PAD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="2,54/1,0">
<connects>
<connect gate="P" pin="P" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:30828/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<groups>
<schematic_group name="LOGO"/>
</groups>
<parts>
<part name="SUPPLY37" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SDA" library="led" library_urn="urn:adsk.eagle:library:259" deviceset="LED" device="CHIPLED_0805" package3d_urn="urn:adsk.eagle:package:15821/2"/>
<part name="RSDA" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="68"/>
<part name="SCL" library="led" library_urn="urn:adsk.eagle:library:259" deviceset="LED" device="CHIPLED_0805" package3d_urn="urn:adsk.eagle:package:15821/2"/>
<part name="RSCL" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="68"/>
<part name="VCC" library="led" library_urn="urn:adsk.eagle:library:259" deviceset="LED" device="CHIPLED_0805" package3d_urn="urn:adsk.eagle:package:15821/2"/>
<part name="RVCC" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="68"/>
<part name="SUPPLY38" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="R4" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="4.7k"/>
<part name="R5" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="4.7k"/>
<part name="FRAME1" library="frames" library_urn="urn:adsk.eagle:library:229" deviceset="A4L-LOC" device=""/>
<part name="S1" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY5" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S2" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY2" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S3" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY8" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S4" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY11" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S5" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY14" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S6" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY17" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S7" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY20" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S8" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY23" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S9" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY26" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S10" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY29" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S11" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY32" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S12" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY35" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S13" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY44" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S14" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY47" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S15" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY50" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="S16" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST6" device="" package3d_urn="urn:adsk.eagle:package:11903788/5"/>
<part name="SUPPLY53" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY1" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY3" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY4" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY6" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY7" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY9" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY10" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY12" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY13" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY15" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY16" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY18" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY19" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY21" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY22" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY24" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY25" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY28" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="J1" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="JST5" device="" package3d_urn="urn:adsk.eagle:package:11903790/5"/>
<part name="SUPPLY31" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY33" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SW1" library="special" library_urn="urn:adsk.eagle:library:367" deviceset="SW_DIP-3" device="" package3d_urn="urn:adsk.eagle:package:26663/2"/>
<part name="R1" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="R2" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="R3" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="SUPPLY34" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY36" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY39" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY40" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="U1" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="MCP23017-E/SP" device="" package3d_urn="urn:adsk.eagle:package:16498237/2"/>
<part name="U2" library="LibBeaconMat" library_urn="urn:adsk.eagle:library:16498234" deviceset="MCP23017-E/SP" device="" package3d_urn="urn:adsk.eagle:package:16498237/2"/>
<part name="SW2" library="special" library_urn="urn:adsk.eagle:library:367" deviceset="SW_DIP-3" device="" package3d_urn="urn:adsk.eagle:package:26663/2"/>
<part name="R6" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="R7" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="R8" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-EU_" device="R0603" package3d_urn="urn:adsk.eagle:package:23555/3" value="10k"/>
<part name="SUPPLY27" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY30" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY41" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY42" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="RESET2" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="2,54/1,0" device="" package3d_urn="urn:adsk.eagle:package:30828/1" override_package3d_urn="urn:adsk.eagle:package:16498393/2" override_package_urn="urn:adsk.eagle:footprint:30810/1"/>
<part name="RESET1" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="2,54/1,0" device="" package3d_urn="urn:adsk.eagle:package:30828/1" override_package3d_urn="urn:adsk.eagle:package:16498394/2" override_package_urn="urn:adsk.eagle:footprint:30810/1"/>
<part name="SUPPLY43" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY45" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY46" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY48" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<rectangle x1="184.30875" y1="5.61975" x2="184.43575" y2="5.68325" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="5.68325" x2="174.27575" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.65675" y1="5.68325" x2="174.91075" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="5.68325" x2="175.54575" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="5.68325" x2="176.18075" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="5.68325" x2="177.00625" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="5.68325" x2="177.89525" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="5.68325" x2="178.91125" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="5.68325" x2="179.48275" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.73675" y1="5.68325" x2="180.18125" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="5.68325" x2="181.13375" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.57825" y1="5.68325" x2="182.02275" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="5.68325" x2="182.78475" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="5.68325" x2="183.48325" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="5.68325" x2="184.49925" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="5.68325" x2="185.19775" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="5.68325" x2="186.27725" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="5.68325" x2="187.86475" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="5.68325" x2="188.24575" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="5.68325" x2="189.13475" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="5.68325" x2="189.76975" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.02375" y1="5.68325" x2="190.27775" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.59525" y1="5.68325" x2="190.91275" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="5.68325" x2="191.42075" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="5.68325" x2="192.75425" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="5.68325" x2="193.13525" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="5.68325" x2="194.02425" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="5.68325" x2="194.65925" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="5.68325" x2="195.73875" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.05625" y1="5.68325" x2="196.50075" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="5.68325" x2="197.00875" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="5.68325" x2="197.58025" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="5.68325" x2="198.15175" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="5.68325" x2="199.10425" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="5.68325" x2="199.54875" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="5.68325" x2="200.69175" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.88225" y1="5.68325" x2="201.13625" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.51725" y1="5.68325" x2="201.77125" y2="5.74675" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="5.74675" x2="174.27575" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.65675" y1="5.74675" x2="174.91075" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.22825" y1="5.74675" x2="175.54575" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="5.74675" x2="176.24425" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="5.74675" x2="177.13325" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="5.74675" x2="178.02225" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="5.74675" x2="178.91125" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="5.74675" x2="179.41925" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="5.74675" x2="180.24475" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="5.74675" x2="181.19725" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.51475" y1="5.74675" x2="182.14975" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="5.74675" x2="182.78475" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="5.74675" x2="183.48325" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="5.74675" x2="184.43575" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="5.74675" x2="185.32475" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="5.74675" x2="186.27725" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="5.74675" x2="187.86475" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="5.74675" x2="188.30925" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="5.74675" x2="189.13475" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="5.74675" x2="189.76975" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.02375" y1="5.74675" x2="190.27775" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.59525" y1="5.74675" x2="190.84925" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="5.74675" x2="191.42075" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="5.74675" x2="192.75425" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="5.74675" x2="193.19875" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="5.74675" x2="194.02425" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="5.74675" x2="194.65925" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="5.74675" x2="195.73875" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.99275" y1="5.74675" x2="196.56425" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="5.74675" x2="197.07225" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="5.74675" x2="197.58025" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="5.74675" x2="198.21525" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="5.74675" x2="199.10425" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="5.74675" x2="199.54875" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="5.74675" x2="200.75525" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.94575" y1="5.74675" x2="201.19975" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.51725" y1="5.74675" x2="201.77125" y2="5.81025" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="5.81025" x2="174.27575" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="5.81025" x2="175.48225" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="5.81025" x2="175.92675" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="5.81025" x2="176.30775" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="5.81025" x2="176.75225" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="5.81025" x2="177.13325" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="5.81025" x2="178.08575" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="5.81025" x2="178.91125" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="5.81025" x2="179.41925" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="5.81025" x2="180.30825" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="5.81025" x2="181.26075" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="5.81025" x2="182.21325" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="5.81025" x2="182.78475" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="5.81025" x2="183.48325" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="5.81025" x2="184.43575" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="5.81025" x2="185.38825" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="5.81025" x2="186.27725" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="5.81025" x2="187.86475" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="5.81025" x2="188.30925" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="5.81025" x2="189.13475" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="5.81025" x2="189.76975" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.02375" y1="5.81025" x2="190.84925" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="5.81025" x2="191.42075" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="5.81025" x2="192.75425" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="5.81025" x2="193.19875" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="5.81025" x2="194.02425" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="5.81025" x2="194.65925" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="5.81025" x2="195.73875" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.92925" y1="5.81025" x2="196.62775" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="5.81025" x2="197.07225" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="5.81025" x2="197.58025" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="5.81025" x2="198.21525" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="5.81025" x2="199.10425" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="5.81025" x2="199.54875" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="5.81025" x2="200.43775" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.50125" y1="5.81025" x2="200.81875" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.94575" y1="5.81025" x2="201.77125" y2="5.87375" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="5.87375" x2="173.13275" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="5.87375" x2="173.89475" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="5.87375" x2="175.48225" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="5.87375" x2="175.86325" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="5.87375" x2="176.30775" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="5.87375" x2="176.68875" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="5.87375" x2="177.19675" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="5.87375" x2="177.57775" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.83175" y1="5.87375" x2="178.08575" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="5.87375" x2="178.91125" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="5.87375" x2="179.35575" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="5.87375" x2="179.86375" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="5.87375" x2="180.37175" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="5.87375" x2="180.75275" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="5.87375" x2="181.26075" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="5.87375" x2="181.70525" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.89575" y1="5.87375" x2="182.21325" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="5.87375" x2="182.78475" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="5.87375" x2="183.48325" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="5.87375" x2="183.92775" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="5.87375" x2="184.43575" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="5.87375" x2="184.88025" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="5.87375" x2="185.38825" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="5.87375" x2="185.83275" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="5.87375" x2="187.48375" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="5.87375" x2="188.30925" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="5.87375" x2="189.13475" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="5.87375" x2="189.76975" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.08725" y1="5.87375" x2="190.78575" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="5.87375" x2="191.48425" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="5.87375" x2="192.30975" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="5.87375" x2="193.19875" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="5.87375" x2="194.02425" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="5.87375" x2="194.65925" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="5.87375" x2="195.29425" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="5.87375" x2="196.18325" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="5.87375" x2="196.62775" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="5.87375" x2="197.07225" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="5.87375" x2="197.58025" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="5.87375" x2="198.21525" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="5.87375" x2="199.10425" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="5.87375" x2="199.54875" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="5.87375" x2="200.37425" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="5.87375" x2="200.81875" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.94575" y1="5.87375" x2="201.70775" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.21575" y1="5.87375" x2="202.53325" y2="5.93725" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="5.93725" x2="173.13275" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="5.93725" x2="173.89475" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="5.93725" x2="175.41875" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="5.93725" x2="175.79975" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="5.93725" x2="176.30775" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="5.93725" x2="176.62525" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="5.93725" x2="177.19675" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="5.93725" x2="177.57775" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.83175" y1="5.93725" x2="178.14925" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="5.93725" x2="178.91125" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="5.93725" x2="179.35575" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="5.93725" x2="179.80025" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="5.93725" x2="180.37175" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="5.93725" x2="180.75275" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="5.93725" x2="181.26075" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="5.93725" x2="181.64175" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.95925" y1="5.93725" x2="182.21325" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="5.93725" x2="182.78475" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="5.93725" x2="183.48325" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="5.93725" x2="183.86425" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="5.93725" x2="184.11825" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="5.93725" x2="184.43575" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="5.93725" x2="184.88025" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="5.93725" x2="185.38825" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="5.93725" x2="185.83275" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="5.93725" x2="187.42025" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="5.93725" x2="188.30925" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="5.93725" x2="189.13475" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="5.93725" x2="189.76975" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.08725" y1="5.93725" x2="190.78575" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="5.93725" x2="191.48425" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="5.93725" x2="192.30975" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="5.93725" x2="193.19875" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="5.93725" x2="194.02425" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="5.93725" x2="194.65925" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="5.93725" x2="195.29425" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="5.93725" x2="196.11975" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="5.93725" x2="196.62775" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="5.93725" x2="197.07225" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="5.93725" x2="197.58025" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="5.93725" x2="198.21525" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="5.93725" x2="199.10425" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="5.93725" x2="199.54875" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="5.93725" x2="200.31075" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="5.93725" x2="200.81875" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="5.93725" x2="201.70775" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.21575" y1="5.93725" x2="202.53325" y2="6.00075" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="6.00075" x2="173.13275" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.00075" x2="173.89475" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="6.00075" x2="175.03775" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.16475" y1="6.00075" x2="175.41875" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="6.00075" x2="176.30775" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="6.00075" x2="177.13325" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="6.00075" x2="177.51425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.89525" y1="6.00075" x2="178.14925" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.00075" x2="179.29225" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="6.00075" x2="179.80025" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="6.00075" x2="180.37175" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.00075" x2="181.26075" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="6.00075" x2="181.64175" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.95925" y1="6.00075" x2="182.21325" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="6.00075" x2="182.78475" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.00075" x2="183.48325" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="6.00075" x2="183.86425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="6.00075" x2="184.49925" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.00075" x2="184.88025" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.00075" x2="185.38825" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.00075" x2="186.21375" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.00075" x2="187.80125" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.00075" x2="188.62675" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.00075" x2="189.13475" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="6.00075" x2="189.76975" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.08725" y1="6.00075" x2="190.34125" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.53175" y1="6.00075" x2="190.78575" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="6.00075" x2="191.48425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.00075" x2="192.69075" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.00075" x2="193.51625" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.00075" x2="194.02425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="6.00075" x2="194.65925" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.00075" x2="195.67525" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="6.00075" x2="196.11975" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.00075" x2="197.58025" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="6.00075" x2="198.27875" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.00075" x2="199.10425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.00075" x2="199.92975" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.37425" y1="6.00075" x2="200.81875" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="6.00075" x2="201.26325" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.39025" y1="6.00075" x2="201.64425" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.21575" y1="6.00075" x2="202.53325" y2="6.06425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.06425" x2="173.89475" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="6.06425" x2="175.03775" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.16475" y1="6.06425" x2="175.41875" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="6.06425" x2="176.30775" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="6.06425" x2="177.13325" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="6.06425" x2="177.51425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.89525" y1="6.06425" x2="178.14925" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.06425" x2="179.35575" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="6.06425" x2="179.80025" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="6.06425" x2="180.37175" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.06425" x2="181.19725" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="6.06425" x2="181.64175" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.95925" y1="6.06425" x2="182.27675" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="6.06425" x2="182.78475" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.06425" x2="183.48325" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="6.06425" x2="183.86425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="6.06425" x2="184.49925" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.06425" x2="184.88025" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.06425" x2="185.38825" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.06425" x2="186.21375" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.06425" x2="187.86475" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.06425" x2="188.69025" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.06425" x2="189.13475" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="6.06425" x2="189.76975" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.15075" y1="6.06425" x2="190.40475" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.46825" y1="6.06425" x2="190.72225" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="6.06425" x2="191.48425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.06425" x2="192.75425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.06425" x2="193.57975" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.06425" x2="194.02425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="6.06425" x2="194.65925" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.06425" x2="195.67525" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="6.06425" x2="196.11975" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.06425" x2="197.58025" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="6.06425" x2="198.27875" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.06425" x2="199.10425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.06425" x2="199.99325" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="6.06425" x2="200.81875" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.07275" y1="6.06425" x2="201.26325" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.39025" y1="6.06425" x2="201.64425" y2="6.12775" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.12775" x2="173.89475" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.84725" y1="6.12775" x2="175.03775" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.10125" y1="6.12775" x2="175.35525" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="6.12775" x2="176.18075" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="6.12775" x2="177.00625" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="6.12775" x2="177.51425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.89525" y1="6.12775" x2="178.14925" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.12775" x2="179.41925" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="6.12775" x2="179.80025" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="6.12775" x2="180.37175" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.12775" x2="181.19725" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="6.12775" x2="181.64175" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.95925" y1="6.12775" x2="182.21325" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="6.12775" x2="182.78475" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.12775" x2="183.48325" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="6.12775" x2="183.86425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="6.12775" x2="184.43575" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.12775" x2="184.88025" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.12775" x2="185.38825" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.12775" x2="186.21375" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.12775" x2="187.86475" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.12775" x2="188.30925" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.43625" y1="6.12775" x2="188.69025" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.12775" x2="189.13475" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="6.12775" x2="189.76975" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.15075" y1="6.12775" x2="190.40475" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.46825" y1="6.12775" x2="190.72225" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="6.12775" x2="191.54775" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.12775" x2="192.75425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.12775" x2="193.19875" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="6.12775" x2="193.57975" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.12775" x2="194.02425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="6.12775" x2="194.65925" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.12775" x2="195.67525" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="6.12775" x2="196.11975" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.12775" x2="197.58025" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="6.12775" x2="198.27875" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.12775" x2="199.10425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.12775" x2="199.54875" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.67575" y1="6.12775" x2="199.99325" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="6.12775" x2="200.69175" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.07275" y1="6.12775" x2="201.32675" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.39025" y1="6.12775" x2="201.64425" y2="6.19125" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.19125" x2="173.89475" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.40275" y1="6.19125" x2="174.59325" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.84725" y1="6.19125" x2="175.35525" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="6.19125" x2="175.99025" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="6.19125" x2="176.81575" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="6.19125" x2="177.57775" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.83175" y1="6.19125" x2="178.14925" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.19125" x2="178.91125" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="6.19125" x2="179.41925" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="6.19125" x2="179.80025" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="6.19125" x2="180.37175" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.19125" x2="180.75275" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="6.19125" x2="181.26075" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="6.19125" x2="181.64175" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.95925" y1="6.19125" x2="182.21325" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="6.19125" x2="182.78475" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.19125" x2="183.48325" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="6.19125" x2="183.86425" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="6.19125" x2="184.43575" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.19125" x2="184.88025" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.19125" x2="185.38825" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.19125" x2="185.83275" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.19125" x2="187.42025" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.19125" x2="188.30925" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="6.19125" x2="188.75375" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.19125" x2="189.13475" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="6.19125" x2="189.76975" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.21425" y1="6.19125" x2="190.65875" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="6.19125" x2="191.54775" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.19125" x2="192.30975" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.19125" x2="193.19875" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="6.19125" x2="193.64325" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.19125" x2="194.02425" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="6.19125" x2="194.65925" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.19125" x2="195.29425" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="6.19125" x2="196.11975" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="6.19125" x2="196.56425" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.19125" x2="197.07225" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="6.19125" x2="197.58025" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="6.19125" x2="198.34225" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.19125" x2="199.10425" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.19125" x2="199.54875" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="6.19125" x2="199.99325" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="6.19125" x2="200.50125" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.07275" y1="6.19125" x2="201.58075" y2="6.25475" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.25475" x2="173.89475" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.33925" y1="6.25475" x2="174.59325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.84725" y1="6.25475" x2="175.29175" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="6.25475" x2="175.86325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="6.25475" x2="176.30775" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="6.25475" x2="176.68875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="6.25475" x2="177.13325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="6.25475" x2="177.64125" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.76825" y1="6.25475" x2="178.08575" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.25475" x2="178.91125" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="6.25475" x2="179.41925" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="6.25475" x2="179.86375" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.99075" y1="6.25475" x2="180.30825" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.25475" x2="180.75275" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="6.25475" x2="181.26075" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="6.25475" x2="181.76875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.89575" y1="6.25475" x2="182.21325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="6.25475" x2="183.03875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.25475" x2="183.48325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="6.25475" x2="183.99125" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="6.25475" x2="184.43575" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.25475" x2="184.88025" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.25475" x2="185.38825" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.25475" x2="185.83275" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.25475" x2="187.48375" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.25475" x2="188.30925" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.43625" y1="6.25475" x2="188.75375" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.25475" x2="189.13475" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="6.25475" x2="190.02375" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.21425" y1="6.25475" x2="190.65875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="6.25475" x2="191.54775" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.25475" x2="192.37325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.25475" x2="193.19875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="6.25475" x2="193.64325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.25475" x2="194.02425" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="6.25475" x2="194.91325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.25475" x2="195.29425" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="6.25475" x2="196.18325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="6.25475" x2="196.62775" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.25475" x2="197.07225" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="6.25475" x2="197.58025" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="6.25475" x2="198.34225" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.25475" x2="199.10425" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.25475" x2="199.54875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="6.25475" x2="199.99325" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="6.25475" x2="200.37425" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="6.25475" x2="200.81875" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.13625" y1="6.25475" x2="201.58075" y2="6.31825" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.31825" x2="173.89475" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.33925" y1="6.31825" x2="174.59325" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.91075" y1="6.31825" x2="175.29175" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="6.31825" x2="176.30775" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="6.31825" x2="177.13325" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="6.31825" x2="178.08575" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.31825" x2="179.41925" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="6.31825" x2="180.30825" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.31825" x2="181.19725" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="6.31825" x2="182.14975" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="6.31825" x2="183.10225" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.31825" x2="183.48325" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="6.31825" x2="184.37225" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.31825" x2="184.88025" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.31825" x2="185.38825" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.31825" x2="186.27725" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.31825" x2="187.86475" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.31825" x2="188.69025" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.31825" x2="189.13475" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="6.31825" x2="190.02375" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.21425" y1="6.31825" x2="190.65875" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="6.31825" x2="191.54775" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.31825" x2="192.75425" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.31825" x2="193.57975" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.31825" x2="194.02425" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="6.31825" x2="194.91325" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.31825" x2="195.73875" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.92925" y1="6.31825" x2="196.62775" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.31825" x2="197.07225" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="6.31825" x2="197.58025" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="6.31825" x2="198.34225" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.31825" x2="199.10425" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.31825" x2="199.99325" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="6.31825" x2="200.81875" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.13625" y1="6.31825" x2="201.51725" y2="6.38175" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.38175" x2="173.89475" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.33925" y1="6.38175" x2="174.59325" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.91075" y1="6.38175" x2="175.29175" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="6.38175" x2="176.24425" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="6.38175" x2="177.06975" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="6.38175" x2="178.02225" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.38175" x2="179.35575" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="6.38175" x2="180.24475" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="6.38175" x2="181.19725" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.51475" y1="6.38175" x2="182.08625" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="6.38175" x2="183.10225" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="6.38175" x2="183.48325" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="6.38175" x2="184.30875" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.38175" x2="184.88025" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="6.38175" x2="185.38825" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.38175" x2="186.27725" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="6.38175" x2="187.86475" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="6.38175" x2="188.69025" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.38175" x2="189.13475" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="6.38175" x2="190.02375" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.27775" y1="6.38175" x2="190.59525" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.48425" y1="6.38175" x2="191.61125" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="6.38175" x2="192.75425" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="6.38175" x2="193.57975" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.38175" x2="194.02425" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="6.38175" x2="194.91325" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.38175" x2="195.73875" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.99275" y1="6.38175" x2="196.56425" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="6.38175" x2="197.07225" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="6.38175" x2="197.58025" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="6.38175" x2="198.34225" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.38175" x2="199.10425" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.38175" x2="199.92975" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="6.38175" x2="200.75525" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="6.38175" x2="201.51725" y2="6.44525" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="6.44525" x2="173.83125" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.40275" y1="6.44525" x2="174.59325" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.97425" y1="6.44525" x2="175.22825" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="6.44525" x2="176.11725" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="6.44525" x2="176.94275" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="6.44525" x2="177.83175" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="6.44525" x2="179.16525" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="6.44525" x2="180.11775" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.56225" y1="6.44525" x2="181.07025" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="6.44525" x2="181.95925" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="6.44525" x2="183.03875" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="6.44525" x2="183.41975" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.86425" y1="6.44525" x2="184.18175" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="6.44525" x2="184.81675" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="6.44525" x2="185.38825" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="6.44525" x2="186.21375" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="6.44525" x2="187.86475" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="6.44525" x2="188.49975" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="6.44525" x2="189.07125" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="6.44525" x2="190.02375" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.34125" y1="6.44525" x2="190.53175" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.48425" y1="6.44525" x2="191.61125" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="6.44525" x2="192.75425" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="6.44525" x2="193.38925" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="6.44525" x2="194.02425" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="6.44525" x2="194.91325" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="6.44525" x2="195.67525" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.11975" y1="6.44525" x2="196.43725" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="6.44525" x2="197.00875" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="6.44525" x2="197.58025" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.27875" y1="6.44525" x2="198.34225" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="6.44525" x2="199.10425" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="6.44525" x2="199.80275" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.31075" y1="6.44525" x2="200.62825" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="6.44525" x2="201.45375" y2="6.50875" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.08025" x2="175.22825" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="7.08025" x2="177.51425" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="7.08025" x2="180.75275" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.08025" x2="184.49925" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="7.08025" x2="186.84875" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="7.08025" x2="189.83325" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.08025" x2="193.64325" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="7.08025" x2="195.04025" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.18325" y1="7.08025" x2="197.19925" y2="7.14375" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.14375" x2="175.22825" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="7.14375" x2="177.57775" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="7.14375" x2="180.94325" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.14375" x2="184.49925" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="7.14375" x2="187.03925" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.14375" x2="189.83325" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.14375" x2="193.64325" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.14375" x2="195.04025" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.11975" y1="7.14375" x2="197.19925" y2="7.20725" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.20725" x2="175.22825" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="7.20725" x2="177.57775" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="7.20725" x2="181.07025" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.20725" x2="184.49925" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="7.20725" x2="187.10275" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.20725" x2="189.83325" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.20725" x2="193.64325" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.20725" x2="195.04025" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.05625" y1="7.20725" x2="197.13575" y2="7.27075" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.27075" x2="175.22825" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="7.27075" x2="177.64125" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="7.27075" x2="181.19725" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.27075" x2="184.49925" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="7.27075" x2="187.22975" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.27075" x2="189.83325" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.27075" x2="193.64325" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.27075" x2="195.04025" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.05625" y1="7.27075" x2="197.13575" y2="7.33425" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.33425" x2="175.22825" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="7.33425" x2="177.64125" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="7.33425" x2="181.26075" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.33425" x2="184.49925" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="7.33425" x2="187.29325" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.33425" x2="189.83325" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.33425" x2="193.64325" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.33425" x2="195.04025" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.99275" y1="7.33425" x2="197.07225" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="7.33425" x2="199.73925" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.34275" y1="7.33425" x2="202.46975" y2="7.39775" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.39775" x2="175.22825" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="7.39775" x2="177.70475" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="7.39775" x2="181.32425" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.39775" x2="184.49925" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="7.39775" x2="187.35675" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.39775" x2="189.83325" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.39775" x2="193.64325" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.39775" x2="195.04025" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.99275" y1="7.39775" x2="197.07225" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.48525" y1="7.39775" x2="199.73925" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.15225" y1="7.39775" x2="202.46975" y2="7.46125" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.46125" x2="175.22825" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="7.46125" x2="177.70475" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="7.46125" x2="181.38775" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.46125" x2="184.49925" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="7.46125" x2="187.42025" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.46125" x2="189.83325" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.46125" x2="193.64325" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.46125" x2="195.04025" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.92925" y1="7.46125" x2="197.00875" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="7.46125" x2="199.73925" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.96175" y1="7.46125" x2="202.46975" y2="7.52475" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.52475" x2="175.22825" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="7.52475" x2="177.70475" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="7.52475" x2="181.45125" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.52475" x2="184.49925" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.00725" y1="7.52475" x2="187.48375" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.52475" x2="189.83325" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.52475" x2="193.64325" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.52475" x2="195.04025" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.92925" y1="7.52475" x2="196.94525" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.10425" y1="7.52475" x2="199.73925" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.83475" y1="7.52475" x2="202.46975" y2="7.58825" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.58825" x2="175.22825" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="7.58825" x2="177.76825" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="7.58825" x2="181.51475" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.58825" x2="184.49925" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="7.58825" x2="187.48375" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.58825" x2="189.83325" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.58825" x2="193.64325" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.58825" x2="195.04025" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="7.58825" x2="196.94525" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.97725" y1="7.58825" x2="199.73925" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.64425" y1="7.58825" x2="202.46975" y2="7.65175" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.65175" x2="175.22825" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="7.65175" x2="177.76825" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="7.65175" x2="180.11775" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="7.65175" x2="181.51475" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.65175" x2="184.49925" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="7.65175" x2="187.54725" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.65175" x2="189.83325" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.65175" x2="193.64325" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.65175" x2="195.04025" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="7.65175" x2="196.88175" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.78675" y1="7.65175" x2="199.73925" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.51725" y1="7.65175" x2="202.46975" y2="7.71525" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.71525" x2="173.70425" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="7.71525" x2="177.83175" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="7.71525" x2="179.86375" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="7.71525" x2="181.57825" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.71525" x2="184.49925" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="7.71525" x2="185.95975" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.53125" y1="7.71525" x2="187.54725" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.71525" x2="189.83325" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.71525" x2="192.11925" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.71525" x2="195.04025" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.80225" y1="7.71525" x2="196.81825" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.65975" y1="7.71525" x2="199.73925" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.32675" y1="7.71525" x2="202.46975" y2="7.77875" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.77875" x2="173.70425" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="7.77875" x2="177.83175" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="7.77875" x2="179.80025" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.62575" y1="7.77875" x2="181.57825" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.77875" x2="183.10225" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="7.77875" x2="185.89625" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.59475" y1="7.77875" x2="187.54725" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.77875" x2="189.83325" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.77875" x2="192.11925" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.77875" x2="195.10375" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.73875" y1="7.77875" x2="196.81825" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="7.77875" x2="199.73925" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="7.77875" x2="202.46975" y2="7.84225" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.84225" x2="173.70425" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.24425" y1="7.84225" x2="177.83175" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="7.84225" x2="179.73675" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="7.84225" x2="181.64175" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.84225" x2="183.10225" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="7.84225" x2="185.83275" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.65825" y1="7.84225" x2="187.61075" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.84225" x2="189.83325" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.84225" x2="192.11925" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.84225" x2="195.16725" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.73875" y1="7.84225" x2="196.75475" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.34225" y1="7.84225" x2="199.73925" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="7.84225" x2="202.46975" y2="7.90575" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.90575" x2="173.70425" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.24425" y1="7.90575" x2="177.89525" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="7.90575" x2="179.67325" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="7.90575" x2="181.64175" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.90575" x2="183.10225" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="7.90575" x2="185.76925" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="7.90575" x2="187.61075" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.90575" x2="189.83325" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.90575" x2="192.11925" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.90575" x2="195.23075" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.67525" y1="7.90575" x2="196.69125" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="7.90575" x2="199.73925" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.88225" y1="7.90575" x2="202.46975" y2="7.96925" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="7.96925" x2="173.70425" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.18075" y1="7.96925" x2="177.00625" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="7.96925" x2="177.89525" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="7.96925" x2="179.67325" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="7.96925" x2="181.64175" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="7.96925" x2="183.10225" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="7.96925" x2="185.76925" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="7.96925" x2="187.61075" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="7.96925" x2="189.83325" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="7.96925" x2="192.11925" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="7.96925" x2="195.29425" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.67525" y1="7.96925" x2="196.69125" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="7.96925" x2="199.67575" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.69175" y1="7.96925" x2="202.34275" y2="8.03275" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.03275" x2="173.70425" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.18075" y1="8.03275" x2="177.00625" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="8.03275" x2="177.95875" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.03275" x2="179.60975" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="8.03275" x2="181.64175" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.03275" x2="183.10225" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.03275" x2="185.76925" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.03275" x2="187.61075" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.03275" x2="189.83325" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.03275" x2="192.11925" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.03275" x2="195.35775" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.61175" y1="8.03275" x2="196.62775" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="8.03275" x2="199.48525" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.50125" y1="8.03275" x2="202.21575" y2="8.09625" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.09625" x2="173.70425" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.18075" y1="8.09625" x2="177.00625" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="8.09625" x2="177.95875" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.09625" x2="179.60975" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="8.09625" x2="181.70525" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.09625" x2="183.10225" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.09625" x2="185.70575" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.09625" x2="187.61075" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.09625" x2="189.83325" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.09625" x2="192.11925" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.09625" x2="195.42125" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.61175" y1="8.09625" x2="196.56425" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="8.09625" x2="199.29475" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.37425" y1="8.09625" x2="202.02525" y2="8.15975" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.15975" x2="173.70425" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.11725" y1="8.15975" x2="176.94275" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.19675" y1="8.15975" x2="177.95875" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.15975" x2="179.60975" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.15975" x2="181.70525" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.15975" x2="183.10225" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.15975" x2="185.70575" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.15975" x2="187.61075" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.15975" x2="189.83325" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.15975" x2="192.11925" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.15975" x2="195.48475" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.54825" y1="8.15975" x2="196.56425" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.15975" x2="199.10425" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.15975" x2="201.83475" y2="8.22325" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.22325" x2="173.70425" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.11725" y1="8.22325" x2="176.94275" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.19675" y1="8.22325" x2="178.02225" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.22325" x2="179.60975" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.22325" x2="181.70525" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.22325" x2="183.10225" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.22325" x2="185.70575" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.22325" x2="187.61075" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.22325" x2="189.83325" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.22325" x2="192.11925" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.22325" x2="196.50075" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.22325" x2="198.97725" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.22325" x2="201.64425" y2="8.28675" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.28675" x2="175.10125" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="8.28675" x2="176.94275" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.19675" y1="8.28675" x2="178.02225" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.28675" x2="179.60975" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.28675" x2="181.70525" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.28675" x2="183.10225" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.28675" x2="185.70575" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.28675" x2="187.61075" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.28675" x2="189.83325" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.28675" x2="193.51625" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.28675" x2="196.43725" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.28675" x2="198.78675" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.28675" x2="201.45375" y2="8.35025" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.35025" x2="175.10125" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="8.35025" x2="176.87925" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="8.35025" x2="178.02225" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="8.35025" x2="179.60975" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.35025" x2="181.70525" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.35025" x2="183.10225" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.35025" x2="185.70575" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.35025" x2="187.61075" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.35025" x2="189.83325" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.35025" x2="193.51625" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.35025" x2="196.43725" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.35025" x2="198.59625" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.35025" x2="201.32675" y2="8.41375" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.41375" x2="175.10125" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="8.41375" x2="176.87925" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="8.41375" x2="178.08575" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="8.41375" x2="179.60975" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.41375" x2="181.70525" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.41375" x2="183.10225" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.41375" x2="185.70575" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.41375" x2="187.61075" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.41375" x2="189.83325" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.41375" x2="193.51625" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.41375" x2="196.37375" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.41375" x2="198.40575" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.41375" x2="201.13625" y2="8.47725" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.47725" x2="175.10125" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="8.47725" x2="176.87925" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="8.47725" x2="178.08575" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="8.47725" x2="179.60975" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.47725" x2="181.70525" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.47725" x2="183.10225" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.47725" x2="185.70575" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.47725" x2="187.61075" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.47725" x2="189.83325" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.47725" x2="193.51625" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.47725" x2="196.31025" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.47725" x2="198.46925" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.47725" x2="201.13625" y2="8.54075" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.54075" x2="175.10125" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="8.54075" x2="176.81575" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="8.54075" x2="178.14925" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="8.54075" x2="179.60975" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.54075" x2="181.70525" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.54075" x2="183.10225" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.54075" x2="185.70575" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.54075" x2="187.61075" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.54075" x2="189.83325" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.54075" x2="193.51625" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.54075" x2="196.31025" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.54075" x2="198.65975" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.54075" x2="201.32675" y2="8.60425" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.60425" x2="175.10125" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="8.60425" x2="176.81575" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="8.60425" x2="178.14925" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.60425" x2="179.60975" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.60425" x2="181.70525" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.60425" x2="183.10225" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.60425" x2="185.70575" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.60425" x2="187.61075" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.60425" x2="189.83325" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.60425" x2="193.51625" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.60425" x2="196.24675" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.60425" x2="198.78675" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.60425" x2="201.51725" y2="8.66775" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.66775" x2="175.10125" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="8.66775" x2="176.81575" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="8.66775" x2="178.14925" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.66775" x2="179.60975" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.66775" x2="181.70525" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.66775" x2="183.10225" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.66775" x2="185.70575" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.66775" x2="187.61075" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.66775" x2="189.83325" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.66775" x2="193.51625" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.66775" x2="196.24675" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.66775" x2="198.97725" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="8.66775" x2="201.70775" y2="8.73125" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.73125" x2="175.10125" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="8.73125" x2="176.75225" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="8.73125" x2="178.21275" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.73125" x2="179.60975" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="8.73125" x2="181.70525" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.73125" x2="183.10225" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.73125" x2="185.70575" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.73125" x2="187.61075" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.73125" x2="189.83325" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.73125" x2="193.51625" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.73125" x2="196.18325" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="8.73125" x2="199.16775" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="8.73125" x2="201.83475" y2="8.79475" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.79475" x2="173.70425" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="8.79475" x2="176.75225" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="8.79475" x2="178.21275" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.79475" x2="179.60975" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="8.79475" x2="181.70525" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.79475" x2="183.10225" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.79475" x2="185.70575" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.79475" x2="187.61075" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.79475" x2="189.83325" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.79475" x2="192.11925" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.79475" x2="196.18325" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="8.79475" x2="199.35825" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.37425" y1="8.79475" x2="202.02525" y2="8.85825" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.85825" x2="173.70425" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="8.85825" x2="176.75225" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="8.85825" x2="178.27625" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.85825" x2="179.60975" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="8.85825" x2="181.64175" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.85825" x2="183.10225" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.85825" x2="185.70575" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.85825" x2="187.61075" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.85825" x2="189.83325" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.85825" x2="192.11925" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.85825" x2="195.04025" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.16725" y1="8.85825" x2="196.24675" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="8.85825" x2="199.48525" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="8.85825" x2="202.21575" y2="8.92175" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.92175" x2="173.70425" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="8.92175" x2="176.68875" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.45075" y1="8.92175" x2="178.27625" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="8.92175" x2="179.67325" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="8.92175" x2="181.64175" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.92175" x2="183.10225" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.92175" x2="185.70575" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.92175" x2="187.61075" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.92175" x2="189.83325" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.92175" x2="192.11925" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.92175" x2="195.04025" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.23075" y1="8.92175" x2="196.31025" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="8.92175" x2="199.67575" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.69175" y1="8.92175" x2="202.40625" y2="8.98525" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="8.98525" x2="173.70425" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="8.98525" x2="176.68875" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.45075" y1="8.98525" x2="178.27625" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="8.98525" x2="179.67325" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="8.98525" x2="181.64175" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="8.98525" x2="183.10225" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="8.98525" x2="185.70575" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="8.98525" x2="187.61075" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="8.98525" x2="189.83325" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="8.98525" x2="192.11925" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="8.98525" x2="195.04025" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.23075" y1="8.98525" x2="196.37375" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="8.98525" x2="199.73925" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.88225" y1="8.98525" x2="202.46975" y2="9.04875" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.04875" x2="173.70425" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="9.04875" x2="176.68875" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.45075" y1="9.04875" x2="178.33975" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="9.04875" x2="179.73675" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.62575" y1="9.04875" x2="181.64175" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.04875" x2="183.10225" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.04875" x2="185.70575" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.04875" x2="187.61075" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="9.04875" x2="189.83325" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.04875" x2="192.11925" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.04875" x2="195.04025" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.29425" y1="9.04875" x2="196.43725" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.34225" y1="9.04875" x2="199.73925" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="9.04875" x2="202.46975" y2="9.11225" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.11225" x2="173.70425" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="9.11225" x2="176.62525" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="9.11225" x2="178.33975" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="9.11225" x2="179.80025" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.56225" y1="9.11225" x2="181.57825" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.11225" x2="183.10225" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.11225" x2="185.70575" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.11225" x2="187.61075" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="9.11225" x2="189.83325" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.11225" x2="192.11925" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.11225" x2="195.04025" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.35775" y1="9.11225" x2="196.50075" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="9.11225" x2="199.73925" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="9.11225" x2="202.46975" y2="9.17575" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.17575" x2="173.70425" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="9.17575" x2="176.62525" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="9.17575" x2="178.40325" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="9.17575" x2="179.92725" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="9.17575" x2="181.57825" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.17575" x2="183.10225" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.17575" x2="185.70575" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.17575" x2="187.61075" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.17575" x2="190.78575" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.17575" x2="192.11925" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.17575" x2="195.04025" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.42125" y1="9.17575" x2="196.56425" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.65975" y1="9.17575" x2="199.73925" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.39025" y1="9.17575" x2="202.46975" y2="9.23925" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.23925" x2="175.16475" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="9.23925" x2="176.62525" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="9.23925" x2="178.40325" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="9.23925" x2="181.51475" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.23925" x2="183.10225" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.23925" x2="185.70575" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.23925" x2="187.61075" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.23925" x2="190.78575" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.23925" x2="193.57975" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.23925" x2="195.04025" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.48475" y1="9.23925" x2="196.62775" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="9.23925" x2="199.73925" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.51725" y1="9.23925" x2="202.46975" y2="9.30275" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.30275" x2="175.22825" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="9.30275" x2="176.56175" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.57775" y1="9.30275" x2="178.40325" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="9.30275" x2="181.45125" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.30275" x2="183.10225" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.30275" x2="185.70575" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.30275" x2="187.61075" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.30275" x2="190.78575" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.30275" x2="193.64325" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.30275" x2="195.04025" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.54825" y1="9.30275" x2="196.69125" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.97725" y1="9.30275" x2="199.73925" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.70775" y1="9.30275" x2="202.46975" y2="9.36625" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.36625" x2="175.22825" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="9.36625" x2="176.56175" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.57775" y1="9.36625" x2="178.46675" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="9.36625" x2="181.45125" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.36625" x2="183.10225" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.36625" x2="185.70575" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.36625" x2="187.61075" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.36625" x2="190.78575" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.36625" x2="193.64325" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.36625" x2="195.04025" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.61175" y1="9.36625" x2="196.75475" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.16775" y1="9.36625" x2="199.73925" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.83475" y1="9.36625" x2="202.46975" y2="9.42975" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.42975" x2="175.22825" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="9.42975" x2="176.56175" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.57775" y1="9.42975" x2="178.46675" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="9.42975" x2="181.38775" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.42975" x2="183.10225" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.42975" x2="185.70575" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.42975" x2="187.61075" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.42975" x2="190.78575" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.42975" x2="193.64325" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.42975" x2="195.04025" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.67525" y1="9.42975" x2="196.81825" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="9.42975" x2="199.73925" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.02525" y1="9.42975" x2="202.46975" y2="9.49325" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.49325" x2="175.22825" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="9.49325" x2="176.49825" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.64125" y1="9.49325" x2="178.53025" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="9.49325" x2="181.32425" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.49325" x2="183.10225" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.49325" x2="185.70575" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.49325" x2="187.61075" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.49325" x2="190.78575" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.49325" x2="193.64325" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.49325" x2="195.04025" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.73875" y1="9.49325" x2="196.88175" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.48525" y1="9.49325" x2="199.73925" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.15225" y1="9.49325" x2="202.46975" y2="9.55675" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.55675" x2="175.22825" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.54575" y1="9.55675" x2="176.49825" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.64125" y1="9.55675" x2="178.53025" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="9.55675" x2="181.26075" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.55675" x2="183.10225" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.55675" x2="185.70575" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.55675" x2="187.61075" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.55675" x2="190.78575" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.55675" x2="193.64325" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.55675" x2="195.04025" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.80225" y1="9.55675" x2="196.94525" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="9.55675" x2="199.73925" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.34275" y1="9.55675" x2="202.46975" y2="9.62025" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.62025" x2="175.22825" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.54575" y1="9.62025" x2="176.49825" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.64125" y1="9.62025" x2="178.53025" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="9.62025" x2="181.13375" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.62025" x2="183.10225" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.62025" x2="185.70575" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.62025" x2="187.61075" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.62025" x2="190.78575" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.62025" x2="193.64325" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.62025" x2="195.04025" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="9.62025" x2="197.00875" y2="9.68375" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.68375" x2="175.22825" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.54575" y1="9.68375" x2="176.43475" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.70475" y1="9.68375" x2="178.59375" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="9.68375" x2="181.07025" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.68375" x2="183.10225" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.68375" x2="185.70575" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.68375" x2="187.61075" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.68375" x2="190.78575" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.68375" x2="193.64325" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.68375" x2="195.04025" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.86575" y1="9.68375" x2="197.07225" y2="9.74725" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.74725" x2="175.22825" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.48225" y1="9.74725" x2="176.43475" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.70475" y1="9.74725" x2="178.59375" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="9.74725" x2="180.87975" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.74725" x2="183.10225" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="9.74725" x2="185.70575" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.72175" y1="9.74725" x2="187.61075" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.74725" x2="190.78575" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.74725" x2="193.64325" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="9.74725" x2="195.04025" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.92925" y1="9.74725" x2="197.13575" y2="9.81075" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="9.81075" x2="175.16475" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.48225" y1="9.81075" x2="176.37125" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.76825" y1="9.81075" x2="178.59375" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="9.81075" x2="180.68925" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="9.81075" x2="183.10225" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="9.81075" x2="185.70575" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.78525" y1="9.81075" x2="187.61075" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="9.81075" x2="190.78575" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="9.81075" x2="193.57975" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="9.81075" x2="195.04025" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.99275" y1="9.81075" x2="197.13575" y2="9.87425" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.75175" y1="10.06475" x2="202.34275" y2="10.12825" layer="94" grouprefs="LOGO"/>
<rectangle x1="172.81525" y1="10.12825" x2="202.34275" y2="10.19175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.29425" y1="10.31875" x2="195.99275" y2="10.38225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="10.31875" x2="198.15175" y2="10.38225" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="10.38225" x2="189.45225" y2="10.44575" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.89675" y1="10.38225" x2="190.34125" y2="10.44575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="10.38225" x2="194.84975" y2="10.44575" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="10.38225" x2="195.99275" y2="10.44575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.38225" x2="198.21525" y2="10.44575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="10.44575" x2="188.69025" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="10.44575" x2="190.15075" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.34125" y1="10.44575" x2="190.40475" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="10.44575" x2="195.99275" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.44575" x2="196.81825" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="10.44575" x2="198.21525" y2="10.50925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="10.50925" x2="186.08675" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="10.50925" x2="187.80125" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="10.50925" x2="188.49975" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="10.50925" x2="188.75375" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="10.50925" x2="189.89675" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.34125" y1="10.50925" x2="190.40475" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="10.50925" x2="195.99275" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.50925" x2="196.75475" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="10.50925" x2="198.21525" y2="10.57275" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.57275" x2="176.11725" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="10.57275" x2="177.19675" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="10.57275" x2="180.81625" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.70525" y1="10.57275" x2="181.76875" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="10.57275" x2="185.32475" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.08675" y1="10.57275" x2="186.15025" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="10.57275" x2="188.30925" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="10.57275" x2="188.81725" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="10.57275" x2="189.19825" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.15075" y1="10.57275" x2="190.21425" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="10.57275" x2="195.86575" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.57275" x2="196.75475" y2="10.63625" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="10.63625" x2="174.21225" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.63625" x2="176.11725" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="10.63625" x2="177.19675" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="10.63625" x2="177.32375" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="10.63625" x2="180.87975" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.76875" y1="10.63625" x2="181.83225" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="10.63625" x2="183.22925" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="10.63625" x2="183.41975" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="10.63625" x2="184.94375" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.02325" y1="10.63625" x2="186.08675" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="10.63625" x2="187.67425" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="10.63625" x2="188.05525" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="10.63625" x2="188.75375" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="10.63625" x2="189.19825" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.96025" y1="10.63625" x2="190.02375" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.73825" y1="10.63625" x2="192.24625" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.34175" y1="10.63625" x2="195.67525" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.63625" x2="196.75475" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="10.63625" x2="197.58025" y2="10.69975" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.57725" y1="10.69975" x2="174.08525" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.27575" y1="10.69975" x2="174.33925" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.69975" x2="174.91075" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="10.69975" x2="176.05375" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="10.69975" x2="177.13325" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="10.69975" x2="177.38725" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="10.69975" x2="179.03825" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="10.69975" x2="180.18125" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="10.69975" x2="180.56225" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.70525" y1="10.69975" x2="181.76875" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.84825" y1="10.69975" x2="183.03875" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="10.69975" x2="183.35625" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="10.69975" x2="184.81675" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="10.69975" x2="187.61075" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.43625" y1="10.69975" x2="188.49975" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="10.69975" x2="189.26175" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.83325" y1="10.69975" x2="189.89675" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.61125" y1="10.69975" x2="192.24625" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.34175" y1="10.69975" x2="195.48475" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="10.69975" x2="196.81825" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="10.69975" x2="197.58025" y2="10.76325" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="10.76325" x2="173.76775" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.76325" x2="174.91075" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="10.76325" x2="177.13325" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="10.76325" x2="177.45075" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="10.76325" x2="179.16525" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="10.76325" x2="180.18125" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="10.76325" x2="180.49875" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.51475" y1="10.76325" x2="181.57825" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="10.76325" x2="182.97525" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="10.76325" x2="183.35625" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="10.76325" x2="184.68975" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="10.76325" x2="185.83275" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="10.76325" x2="187.61075" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="10.76325" x2="187.86475" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.30925" y1="10.76325" x2="188.37275" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="10.76325" x2="189.26175" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="10.76325" x2="189.51575" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.76975" y1="10.76325" x2="189.83325" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.61125" y1="10.76325" x2="192.24625" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.34175" y1="10.76325" x2="195.29425" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="10.76325" x2="196.88175" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="10.76325" x2="197.51675" y2="10.82675" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="10.82675" x2="173.70425" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="10.82675" x2="173.95825" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.82675" x2="174.91075" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="10.82675" x2="175.41875" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="10.82675" x2="177.06975" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="10.82675" x2="177.45075" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.65725" y1="10.82675" x2="179.03825" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="10.82675" x2="180.11775" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="10.82675" x2="180.56225" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.32425" y1="10.82675" x2="181.38775" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="10.82675" x2="182.84825" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="10.82675" x2="183.29275" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="10.82675" x2="184.68975" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="10.82675" x2="185.64225" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="10.82675" x2="187.67425" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="10.82675" x2="187.80125" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="10.82675" x2="187.92825" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="10.82675" x2="188.30925" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="10.82675" x2="189.38875" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.64275" y1="10.82675" x2="189.76975" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.54775" y1="10.82675" x2="192.18275" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="10.82675" x2="195.10375" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="10.82675" x2="196.88175" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="10.82675" x2="197.32625" y2="10.89025" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.38675" y1="10.89025" x2="173.64075" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.83125" y1="10.89025" x2="173.89475" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.89025" x2="174.91075" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="10.89025" x2="175.41875" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="10.89025" x2="177.00625" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="10.89025" x2="177.45075" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="10.89025" x2="178.97475" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="10.89025" x2="179.86375" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="10.89025" x2="180.56225" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.62575" y1="10.89025" x2="180.75275" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="10.89025" x2="181.26075" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.59425" y1="10.89025" x2="182.78475" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="10.89025" x2="183.22925" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="10.89025" x2="184.68975" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="10.89025" x2="185.45175" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="10.89025" x2="187.73775" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="10.89025" x2="188.18225" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="10.89025" x2="189.32525" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="10.89025" x2="189.64275" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.48425" y1="10.89025" x2="191.99225" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="10.89025" x2="194.97675" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="10.89025" x2="196.94525" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="10.89025" x2="197.13575" y2="10.95375" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.38675" y1="10.95375" x2="173.57725" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.76775" y1="10.95375" x2="173.83125" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="10.95375" x2="174.91075" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="10.95375" x2="175.41875" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="10.95375" x2="177.00625" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="10.95375" x2="177.45075" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.72075" y1="10.95375" x2="178.97475" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="10.95375" x2="179.73675" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="10.95375" x2="180.68925" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="10.95375" x2="181.19725" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="10.95375" x2="182.72125" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="10.95375" x2="183.16575" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="10.95375" x2="184.81675" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="10.95375" x2="184.94375" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="10.95375" x2="185.32475" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="10.95375" x2="187.73775" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="10.95375" x2="188.11875" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="10.95375" x2="189.32525" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="10.95375" x2="189.64275" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="10.95375" x2="191.92875" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="10.95375" x2="194.97675" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="10.95375" x2="196.94525" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="10.95375" x2="197.13575" y2="11.01725" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.32325" y1="11.01725" x2="173.57725" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="11.01725" x2="173.76775" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.01725" x2="174.91075" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="11.01725" x2="175.35525" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="11.01725" x2="177.00625" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="11.01725" x2="177.38725" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="11.01725" x2="178.97475" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="11.01725" x2="179.16525" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="11.01725" x2="179.60975" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="11.01725" x2="180.68925" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="11.01725" x2="181.13375" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="11.01725" x2="182.65775" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.03875" y1="11.01725" x2="183.10225" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="11.01725" x2="184.81675" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="11.01725" x2="185.26125" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.01725" x2="187.73775" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.01725" x2="188.05525" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="11.01725" x2="189.32525" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.01725" x2="189.64275" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="11.01725" x2="191.92875" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.34175" y1="11.01725" x2="195.04025" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.01725" x2="195.16725" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="11.01725" x2="196.94525" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="11.01725" x2="197.13575" y2="11.08075" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.25975" y1="11.08075" x2="173.51375" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="11.08075" x2="173.76775" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.08075" x2="174.91075" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="11.08075" x2="175.35525" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.08075" x2="177.00625" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="11.08075" x2="177.38725" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="11.08075" x2="179.10175" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="11.08075" x2="179.54625" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="11.08075" x2="180.68925" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="11.08075" x2="181.07025" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.34025" y1="11.08075" x2="182.59425" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="11.08075" x2="183.03875" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="11.08075" x2="184.75325" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.08075" x2="185.19775" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.08075" x2="187.73775" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="11.08075" x2="187.99175" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="11.08075" x2="189.32525" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.08075" x2="189.64275" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="11.08075" x2="191.86525" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.08075" x2="194.78625" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.08075" x2="195.16725" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.08075" x2="196.94525" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="11.08075" x2="197.13575" y2="11.14425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.19625" y1="11.14425" x2="173.45025" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="11.14425" x2="173.76775" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.14425" x2="174.91075" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="11.14425" x2="175.41875" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.14425" x2="177.00625" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="11.14425" x2="177.38725" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="11.14425" x2="179.03825" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.14425" x2="179.48275" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="11.14425" x2="180.68925" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="11.14425" x2="181.00675" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="11.14425" x2="182.53075" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="11.14425" x2="183.03875" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="11.14425" x2="184.81675" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.14425" x2="185.13425" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.14425" x2="187.73775" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="11.14425" x2="187.99175" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="11.14425" x2="189.32525" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.14425" x2="189.64275" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.03975" y1="11.14425" x2="191.86525" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.14425" x2="194.78625" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.14425" x2="195.16725" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.14425" x2="196.94525" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="11.14425" x2="197.19925" y2="11.20775" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.13275" y1="11.20775" x2="173.45025" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="11.20775" x2="173.76775" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.20775" x2="174.91075" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.20775" x2="175.41875" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.20775" x2="177.00625" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="11.20775" x2="177.38725" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="11.20775" x2="179.03825" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.20775" x2="179.48275" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="11.20775" x2="180.62575" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="11.20775" x2="181.00675" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.14975" y1="11.20775" x2="182.46725" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="11.20775" x2="182.97525" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="11.20775" x2="184.81675" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.20775" x2="185.13425" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.20775" x2="187.67425" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="11.20775" x2="187.99175" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="11.20775" x2="189.26175" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.20775" x2="189.64275" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.03975" y1="11.20775" x2="191.80175" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.20775" x2="194.78625" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.20775" x2="195.16725" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.20775" x2="196.88175" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.20775" x2="197.26275" y2="11.27125" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.13275" y1="11.27125" x2="173.45025" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="11.27125" x2="173.76775" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.27125" x2="174.91075" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.27125" x2="175.41875" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.27125" x2="176.94275" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="11.27125" x2="177.38725" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="11.27125" x2="179.10175" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.27125" x2="179.48275" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="11.27125" x2="180.62575" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.27125" x2="181.00675" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.14975" y1="11.27125" x2="182.46725" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="11.27125" x2="182.97525" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="11.27125" x2="184.81675" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.27125" x2="185.13425" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.27125" x2="187.67425" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.27125" x2="188.05525" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="11.27125" x2="189.26175" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.27125" x2="189.64275" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.97625" y1="11.27125" x2="191.80175" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.27125" x2="194.84975" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.27125" x2="195.16725" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.27125" x2="196.88175" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.27125" x2="197.26275" y2="11.33475" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.19625" y1="11.33475" x2="173.51375" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.76775" y1="11.33475" x2="173.83125" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.33475" x2="174.91075" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.33475" x2="175.41875" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.33475" x2="176.94275" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="11.33475" x2="177.32375" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.33475" x2="179.10175" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="11.33475" x2="179.41925" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="11.33475" x2="180.62575" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.33475" x2="180.94325" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.14975" y1="11.33475" x2="182.53075" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="11.33475" x2="182.97525" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="11.33475" x2="184.81675" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.33475" x2="185.13425" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.33475" x2="187.67425" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.33475" x2="188.05525" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="11.33475" x2="189.26175" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="11.33475" x2="189.64275" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.97625" y1="11.33475" x2="191.73825" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.86525" y1="11.33475" x2="191.99225" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.33475" x2="194.84975" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.33475" x2="195.16725" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.33475" x2="196.88175" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.33475" x2="197.26275" y2="11.39825" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.25975" y1="11.39825" x2="173.57725" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.95825" y1="11.39825" x2="174.02175" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.39825" x2="174.91075" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.39825" x2="175.41875" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="11.39825" x2="176.94275" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="11.39825" x2="177.32375" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.39825" x2="179.16525" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.39825" x2="179.48275" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="11.39825" x2="180.62575" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.39825" x2="180.94325" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="11.39825" x2="182.53075" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="11.39825" x2="182.97525" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="11.39825" x2="184.81675" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.39825" x2="185.13425" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.39825" x2="187.67425" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.39825" x2="188.05525" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="11.39825" x2="189.19825" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.39825" x2="189.64275" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.91275" y1="11.39825" x2="191.73825" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.80175" y1="11.39825" x2="192.05575" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.39825" x2="194.84975" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.39825" x2="195.16725" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="11.39825" x2="196.88175" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.39825" x2="197.26275" y2="11.46175" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.32325" y1="11.46175" x2="173.64075" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.08525" y1="11.46175" x2="174.14875" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.46175" x2="174.91075" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.46175" x2="175.41875" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="11.46175" x2="176.94275" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.19675" y1="11.46175" x2="177.26025" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.46175" x2="179.16525" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.46175" x2="179.48275" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="11.46175" x2="180.56225" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.46175" x2="180.94325" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="11.46175" x2="182.59425" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="11.46175" x2="182.97525" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="11.46175" x2="184.81675" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.46175" x2="185.13425" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.46175" x2="187.67425" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.46175" x2="188.05525" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="11.46175" x2="189.19825" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.46175" x2="189.57925" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.91275" y1="11.46175" x2="192.11925" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.46175" x2="194.91325" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.46175" x2="195.16725" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.46175" x2="196.88175" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.46175" x2="197.26275" y2="11.52525" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.38675" y1="11.52525" x2="173.70425" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.21225" y1="11.52525" x2="174.27575" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.52525" x2="174.91075" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.52525" x2="175.41875" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="11.52525" x2="176.87925" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="11.52525" x2="177.26025" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="11.52525" x2="177.95875" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.52525" x2="179.22875" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.52525" x2="179.48275" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="11.52525" x2="180.56225" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.52525" x2="180.94325" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.34025" y1="11.52525" x2="182.65775" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="11.52525" x2="183.03875" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="11.52525" x2="184.75325" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.52525" x2="185.13425" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.52525" x2="187.67425" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.52525" x2="188.05525" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="11.52525" x2="189.19825" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.52525" x2="189.57925" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.97625" y1="11.52525" x2="192.11925" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.52525" x2="194.91325" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.10375" y1="11.52525" x2="195.16725" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.52525" x2="196.88175" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.52525" x2="197.26275" y2="11.58875" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="11.58875" x2="173.83125" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.27575" y1="11.58875" x2="174.33925" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.58875" x2="174.91075" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.58875" x2="175.41875" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="11.58875" x2="176.87925" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="11.58875" x2="177.19675" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="11.58875" x2="177.76825" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.95875" y1="11.58875" x2="178.02225" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.58875" x2="179.22875" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="11.58875" x2="179.48275" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="11.58875" x2="180.56225" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.58875" x2="180.94325" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.40375" y1="11.58875" x2="182.72125" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.03875" y1="11.58875" x2="183.10225" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="11.58875" x2="184.75325" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="11.58875" x2="185.13425" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="11.58875" x2="187.67425" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.58875" x2="188.05525" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="11.58875" x2="189.13475" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.58875" x2="189.57925" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.10325" y1="11.58875" x2="192.18275" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="11.58875" x2="194.84975" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.58875" x2="195.16725" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.58875" x2="196.88175" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.58875" x2="197.26275" y2="11.65225" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="11.65225" x2="173.89475" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.33925" y1="11.65225" x2="174.40275" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="11.65225" x2="174.97425" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.65225" x2="175.41875" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="11.65225" x2="176.81575" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.06975" y1="11.65225" x2="177.13325" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="11.65225" x2="177.70475" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.95875" y1="11.65225" x2="178.02225" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.65225" x2="179.22875" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="11.65225" x2="179.54625" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="11.65225" x2="180.49875" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.65225" x2="180.94325" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="11.65225" x2="182.78475" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="11.65225" x2="183.16575" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.65225" x2="184.75325" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.65225" x2="185.19775" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.65225" x2="187.61075" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="11.65225" x2="188.05525" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="11.65225" x2="189.13475" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.65225" x2="189.57925" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.16675" y1="11.65225" x2="192.24625" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.65225" x2="194.84975" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.65225" x2="195.10375" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.65225" x2="196.88175" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.65225" x2="197.26275" y2="11.71575" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.57725" y1="11.71575" x2="173.95825" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.40275" y1="11.71575" x2="174.46625" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="11.71575" x2="174.97425" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.71575" x2="175.41875" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="11.71575" x2="176.75225" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.06975" y1="11.71575" x2="177.13325" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.57775" y1="11.71575" x2="177.70475" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.95875" y1="11.71575" x2="178.02225" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.71575" x2="179.22875" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="11.71575" x2="179.54625" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="11.71575" x2="180.49875" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.71575" x2="180.94325" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="11.71575" x2="182.84825" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="11.71575" x2="183.22925" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.71575" x2="184.68975" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.71575" x2="185.19775" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.71575" x2="187.61075" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="11.71575" x2="188.11875" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="11.71575" x2="189.07125" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.71575" x2="189.57925" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="11.71575" x2="192.30975" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.71575" x2="194.91325" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.71575" x2="195.10375" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.71575" x2="196.88175" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.71575" x2="197.26275" y2="11.77925" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="11.77925" x2="174.02175" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.46625" y1="11.77925" x2="174.52975" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="11.77925" x2="174.97425" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.77925" x2="175.41875" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="11.77925" x2="176.68875" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.00625" y1="11.77925" x2="177.06975" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="11.77925" x2="177.64125" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.95875" y1="11.77925" x2="178.02225" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.77925" x2="179.22875" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="11.77925" x2="179.60975" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="11.77925" x2="180.49875" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.77925" x2="180.94325" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.59425" y1="11.77925" x2="182.84825" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="11.77925" x2="183.29275" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.77925" x2="184.68975" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.77925" x2="185.19775" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.77925" x2="187.61075" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="11.77925" x2="188.11875" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="11.77925" x2="189.07125" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.77925" x2="189.57925" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.23025" y1="11.77925" x2="192.30975" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.77925" x2="194.91325" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.77925" x2="195.10375" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.77925" x2="196.88175" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.77925" x2="197.26275" y2="11.84275" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.76775" y1="11.84275" x2="174.08525" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.52975" y1="11.84275" x2="174.59325" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="11.84275" x2="174.97425" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.84275" x2="175.41875" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="11.84275" x2="176.62525" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="11.84275" x2="177.06975" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="11.84275" x2="177.57775" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.89525" y1="11.84275" x2="177.95875" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.84275" x2="179.22875" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="11.84275" x2="179.60975" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="11.84275" x2="180.43525" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.84275" x2="180.94325" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="11.84275" x2="182.84825" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="11.84275" x2="183.35625" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.84275" x2="184.68975" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.84275" x2="185.19775" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.84275" x2="187.61075" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="11.84275" x2="188.11875" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="11.84275" x2="189.07125" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.84275" x2="189.57925" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="11.84275" x2="192.37325" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.84275" x2="194.91325" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.84275" x2="195.10375" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.84275" x2="196.88175" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.84275" x2="197.26275" y2="11.90625" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.83125" y1="11.90625" x2="174.14875" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.52975" y1="11.90625" x2="174.59325" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="11.90625" x2="174.97425" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.90625" x2="175.41875" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="11.90625" x2="176.56175" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="11.90625" x2="177.00625" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.45075" y1="11.90625" x2="177.57775" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.89525" y1="11.90625" x2="177.95875" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.90625" x2="179.22875" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="11.90625" x2="179.67325" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="11.90625" x2="180.43525" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.90625" x2="180.94325" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="11.90625" x2="182.91175" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="11.90625" x2="183.41975" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.90625" x2="184.68975" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="11.90625" x2="185.19775" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.90625" x2="187.61075" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="11.90625" x2="188.11875" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="11.90625" x2="189.00775" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.90625" x2="189.57925" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="11.90625" x2="192.43675" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.90625" x2="194.84975" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.90625" x2="195.10375" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.90625" x2="196.88175" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.90625" x2="197.26275" y2="11.96975" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="11.96975" x2="174.21225" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.59325" y1="11.96975" x2="174.97425" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="11.96975" x2="175.41875" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="11.96975" x2="176.49825" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="11.96975" x2="176.94275" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="11.96975" x2="177.51425" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.83175" y1="11.96975" x2="177.89525" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="11.96975" x2="179.22875" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="11.96975" x2="179.67325" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="11.96975" x2="180.43525" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="11.96975" x2="180.94325" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="11.96975" x2="182.91175" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.41975" y1="11.96975" x2="183.48325" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="11.96975" x2="184.68975" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="11.96975" x2="185.26125" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="11.96975" x2="187.61075" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="11.96975" x2="188.11875" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="11.96975" x2="189.00775" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="11.96975" x2="189.57925" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.35725" y1="11.96975" x2="192.50025" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="11.96975" x2="194.84975" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="11.96975" x2="195.10375" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="11.96975" x2="196.88175" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="11.96975" x2="197.26275" y2="12.03325" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="12.03325" x2="174.91075" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="12.03325" x2="175.41875" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.24425" y1="12.03325" x2="176.43475" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="12.03325" x2="176.94275" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="12.03325" x2="177.51425" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.83175" y1="12.03325" x2="177.89525" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="12.03325" x2="179.16525" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="12.03325" x2="179.73675" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.03325" x2="180.43525" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.03325" x2="181.00675" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="12.03325" x2="182.91175" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="12.03325" x2="183.54675" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="12.03325" x2="184.68975" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="12.03325" x2="185.26125" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.03325" x2="187.54725" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="12.03325" x2="188.11875" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.03325" x2="189.00775" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.03325" x2="189.57925" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.42075" y1="12.03325" x2="192.56375" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="12.03325" x2="194.84975" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="12.03325" x2="195.10375" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.03325" x2="196.88175" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.03325" x2="197.26275" y2="12.09675" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="12.09675" x2="174.78375" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="12.09675" x2="175.41875" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.18075" y1="12.09675" x2="176.37125" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="12.09675" x2="176.87925" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="12.09675" x2="177.45075" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.76825" y1="12.09675" x2="177.83175" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="12.09675" x2="179.16525" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="12.09675" x2="179.73675" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.09675" x2="180.37175" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.09675" x2="181.00675" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="12.09675" x2="182.97525" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.54675" y1="12.09675" x2="183.61025" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="12.09675" x2="184.68975" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="12.09675" x2="185.26125" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.09675" x2="187.54725" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="12.09675" x2="188.11875" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.09675" x2="188.94425" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.09675" x2="189.57925" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.48425" y1="12.09675" x2="192.56375" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="12.09675" x2="194.91325" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="12.09675" x2="195.10375" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.09675" x2="196.88175" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.09675" x2="197.26275" y2="12.16025" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="12.16025" x2="174.65675" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="12.16025" x2="175.41875" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.11725" y1="12.16025" x2="176.30775" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="12.16025" x2="176.87925" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="12.16025" x2="177.38725" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.64125" y1="12.16025" x2="177.70475" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="12.16025" x2="179.16525" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.73675" y1="12.16025" x2="179.80025" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.16025" x2="180.37175" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.16025" x2="181.00675" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="12.16025" x2="182.97525" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="12.16025" x2="183.67375" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="12.16025" x2="184.68975" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="12.16025" x2="185.26125" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.16025" x2="187.54725" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.16025" x2="188.18225" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.16025" x2="188.94425" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.16025" x2="189.57925" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.54775" y1="12.16025" x2="192.62725" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.16025" x2="194.91325" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="12.16025" x2="195.10375" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.16025" x2="196.88175" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.16025" x2="197.26275" y2="12.22375" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.83125" y1="12.22375" x2="174.52975" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="12.22375" x2="175.41875" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.05375" y1="12.22375" x2="176.24425" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="12.22375" x2="177.32375" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.57775" y1="12.22375" x2="177.64125" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="12.22375" x2="179.16525" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="12.22375" x2="179.86375" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.22375" x2="180.37175" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.22375" x2="181.00675" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="12.22375" x2="183.03875" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="12.22375" x2="183.73725" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="12.22375" x2="184.75325" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="12.22375" x2="185.32475" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.22375" x2="187.54725" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.22375" x2="188.18225" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.22375" x2="188.94425" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.22375" x2="189.57925" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.54775" y1="12.22375" x2="192.69075" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.22375" x2="194.91325" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="195.04025" y1="12.22375" x2="195.10375" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.22375" x2="196.88175" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.22375" x2="197.26275" y2="12.28725" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="12.28725" x2="174.40275" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="12.28725" x2="175.41875" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="12.28725" x2="176.18075" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="12.28725" x2="177.26025" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.51425" y1="12.28725" x2="177.57775" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="12.28725" x2="179.22875" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="12.28725" x2="179.86375" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.28725" x2="180.37175" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.28725" x2="181.00675" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="12.28725" x2="183.03875" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="12.28725" x2="183.80075" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="12.28725" x2="184.81675" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="12.28725" x2="185.32475" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.28725" x2="187.61075" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.28725" x2="188.18225" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.28725" x2="188.94425" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.28725" x2="189.57925" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.61125" y1="12.28725" x2="192.75425" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.28725" x2="194.91325" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.28725" x2="195.04025" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.28725" x2="196.88175" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.28725" x2="197.26275" y2="12.35075" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="12.35075" x2="174.27575" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="12.35075" x2="175.35525" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.35075" x2="176.11725" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="12.35075" x2="177.13325" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.38725" y1="12.35075" x2="177.51425" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="12.35075" x2="179.22875" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.86375" y1="12.35075" x2="179.92725" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.35075" x2="180.37175" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.35075" x2="181.00675" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.84825" y1="12.35075" x2="183.16575" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="12.35075" x2="183.86425" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="12.35075" x2="184.81675" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="12.35075" x2="185.32475" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="12.35075" x2="187.61075" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.35075" x2="188.18225" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.35075" x2="188.94425" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.35075" x2="189.57925" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.67475" y1="12.35075" x2="192.75425" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.35075" x2="194.91325" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.35075" x2="195.04025" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.35075" x2="196.88175" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.35075" x2="197.26275" y2="12.41425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="12.41425" x2="174.08525" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.22825" y1="12.41425" x2="175.29175" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="12.41425" x2="176.05375" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="12.41425" x2="176.68875" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.32375" y1="12.41425" x2="177.38725" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="12.41425" x2="179.29225" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.86375" y1="12.41425" x2="179.92725" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.41425" x2="180.37175" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.41425" x2="181.00675" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="12.41425" x2="183.29275" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.86425" y1="12.41425" x2="183.92775" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="12.41425" x2="184.88025" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="12.41425" x2="185.32475" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="12.41425" x2="187.67425" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.41425" x2="188.18225" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.41425" x2="188.94425" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.41425" x2="189.57925" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.73825" y1="12.41425" x2="192.81775" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.41425" x2="194.91325" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.41425" x2="195.04025" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.41425" x2="196.88175" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.41425" x2="197.26275" y2="12.47775" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.57725" y1="12.47775" x2="174.02175" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.16475" y1="12.47775" x2="175.22825" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="12.47775" x2="176.05375" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="12.47775" x2="176.68875" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.26025" y1="12.47775" x2="177.32375" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="12.47775" x2="179.35575" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.92725" y1="12.47775" x2="179.99075" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.47775" x2="180.37175" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.47775" x2="181.00675" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="12.47775" x2="183.41975" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.92775" y1="12.47775" x2="183.99125" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="12.47775" x2="184.94375" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="12.47775" x2="185.38825" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="12.47775" x2="187.73775" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.47775" x2="188.18225" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.47775" x2="188.94425" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.47775" x2="189.57925" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.80175" y1="12.47775" x2="192.88125" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.47775" x2="194.91325" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.47775" x2="195.04025" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.47775" x2="196.88175" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.47775" x2="197.26275" y2="12.54125" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="12.54125" x2="173.89475" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.10125" y1="12.54125" x2="175.22825" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="12.54125" x2="176.05375" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="12.54125" x2="176.62525" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="12.54125" x2="177.19675" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="12.54125" x2="179.41925" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.92725" y1="12.54125" x2="179.99075" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.54125" x2="180.37175" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="12.54125" x2="181.00675" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="12.54125" x2="183.54675" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="12.54125" x2="184.11825" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="12.54125" x2="184.94375" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="12.54125" x2="185.38825" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="12.54125" x2="187.73775" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="12.54125" x2="188.18225" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.54125" x2="188.94425" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.54125" x2="189.57925" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.80175" y1="12.54125" x2="192.94475" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.54125" x2="194.91325" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.54125" x2="195.04025" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.54125" x2="196.88175" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.54125" x2="197.26275" y2="12.60475" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="12.60475" x2="173.83125" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.10125" y1="12.60475" x2="175.16475" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="12.60475" x2="176.05375" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="12.60475" x2="176.62525" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.06975" y1="12.60475" x2="177.13325" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="12.60475" x2="179.48275" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.99075" y1="12.60475" x2="180.05425" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.60475" x2="180.43525" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="12.60475" x2="183.67375" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.05475" y1="12.60475" x2="184.18175" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="12.60475" x2="185.00725" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="12.60475" x2="185.38825" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="12.60475" x2="187.80125" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="12.60475" x2="188.24575" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.60475" x2="188.94425" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.60475" x2="189.57925" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.86525" y1="12.60475" x2="192.94475" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.60475" x2="194.91325" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.60475" x2="195.04025" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.60475" x2="196.88175" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.60475" x2="197.26275" y2="12.66825" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.66825" x2="173.76775" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.03775" y1="12.66825" x2="175.10125" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.66825" x2="176.05375" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="12.66825" x2="176.56175" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="12.66825" x2="177.06975" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="12.66825" x2="179.60975" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="12.66825" x2="180.11775" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.66825" x2="180.43525" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="12.66825" x2="181.07025" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="12.66825" x2="183.80075" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="12.66825" x2="184.18175" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="12.66825" x2="185.00725" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="12.66825" x2="185.38825" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="12.66825" x2="187.86475" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="12.66825" x2="188.24575" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.66825" x2="189.00775" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.66825" x2="189.57925" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.92875" y1="12.66825" x2="192.75425" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="12.66825" x2="193.00825" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="12.66825" x2="194.91325" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.66825" x2="195.04025" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="12.66825" x2="196.88175" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.66825" x2="197.26275" y2="12.73175" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.73175" x2="173.70425" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.97425" y1="12.73175" x2="175.10125" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.73175" x2="176.05375" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="12.73175" x2="176.56175" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="12.73175" x2="176.94275" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="12.73175" x2="180.11775" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="12.73175" x2="180.43525" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="12.73175" x2="181.07025" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="12.73175" x2="183.92775" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="12.73175" x2="184.24525" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="12.73175" x2="185.00725" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="12.73175" x2="185.45175" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="12.73175" x2="187.92825" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="12.73175" x2="188.24575" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.73175" x2="189.00775" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.73175" x2="189.57925" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="12.73175" x2="192.81775" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="12.73175" x2="193.07175" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="12.73175" x2="194.84975" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.73175" x2="195.04025" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="12.73175" x2="196.88175" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.73175" x2="197.26275" y2="12.79525" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.79525" x2="173.70425" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.91075" y1="12.79525" x2="174.97425" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.10125" y1="12.79525" x2="175.16475" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.79525" x2="176.05375" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="12.79525" x2="176.56175" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="12.79525" x2="176.87925" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="12.79525" x2="180.43525" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="12.79525" x2="181.07025" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="12.79525" x2="184.05475" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.24525" y1="12.79525" x2="184.30875" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="12.79525" x2="185.07075" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="12.79525" x2="185.45175" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="12.79525" x2="188.05525" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="12.79525" x2="188.24575" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.79525" x2="189.00775" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.79525" x2="189.57925" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="12.79525" x2="192.88125" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="12.79525" x2="193.13525" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="12.79525" x2="194.84975" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.79525" x2="195.04025" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="12.79525" x2="196.88175" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.79525" x2="197.26275" y2="12.85875" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.85875" x2="173.70425" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.84725" y1="12.85875" x2="174.91075" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.16475" y1="12.85875" x2="175.22825" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.85875" x2="176.05375" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="12.85875" x2="176.49825" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="12.85875" x2="176.75225" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="12.85875" x2="180.43525" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="12.85875" x2="181.07025" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.41975" y1="12.85875" x2="184.37225" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="12.85875" x2="185.07075" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="12.85875" x2="185.45175" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="12.85875" x2="188.18225" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.30925" y1="12.85875" x2="188.37275" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.85875" x2="189.00775" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.85875" x2="189.57925" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="12.85875" x2="192.94475" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="12.85875" x2="193.19875" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="12.85875" x2="194.84975" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.85875" x2="195.04025" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="12.85875" x2="196.81825" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="12.85875" x2="197.38975" y2="12.92225" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.92225" x2="173.70425" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.78375" y1="12.92225" x2="174.84725" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.22825" y1="12.92225" x2="175.29175" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.92225" x2="176.05375" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="12.92225" x2="176.49825" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="12.92225" x2="176.68875" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="12.92225" x2="180.43525" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="12.92225" x2="181.13375" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="12.92225" x2="184.37225" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="12.92225" x2="185.07075" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="12.92225" x2="185.51525" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="12.92225" x2="188.24575" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.30925" y1="12.92225" x2="188.37275" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.92225" x2="189.00775" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.92225" x2="189.57925" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="12.92225" x2="193.00825" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="12.92225" x2="193.19875" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="12.92225" x2="194.84975" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.97675" y1="12.92225" x2="195.04025" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="12.92225" x2="196.81825" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="12.92225" x2="197.38975" y2="12.98575" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="12.98575" x2="173.70425" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="12.98575" x2="174.78375" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.29175" y1="12.98575" x2="175.35525" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="12.98575" x2="176.05375" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="12.98575" x2="176.49825" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="12.98575" x2="176.62525" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="12.98575" x2="180.43525" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="12.98575" x2="181.19725" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.54675" y1="12.98575" x2="184.37225" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="12.98575" x2="185.07075" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="12.98575" x2="185.51525" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="12.98575" x2="188.43625" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="12.98575" x2="189.00775" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="12.98575" x2="189.57925" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="12.98575" x2="193.00825" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="12.98575" x2="193.26225" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="12.98575" x2="194.84975" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="12.98575" x2="195.04025" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="12.98575" x2="196.81825" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="12.98575" x2="197.38975" y2="13.04925" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.45025" y1="13.04925" x2="173.76775" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.59325" y1="13.04925" x2="174.65675" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="13.04925" x2="175.41875" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.04925" x2="176.05375" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.04925" x2="176.49825" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.04925" x2="176.62525" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="13.04925" x2="180.43525" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="13.04925" x2="181.19725" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.54675" y1="13.04925" x2="184.37225" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="13.04925" x2="185.00725" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="13.04925" x2="185.51525" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="13.04925" x2="188.43625" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="13.04925" x2="188.94425" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="13.04925" x2="189.57925" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.24625" y1="13.04925" x2="193.00825" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="13.04925" x2="193.32575" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="13.04925" x2="194.84975" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.04925" x2="194.97675" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.04925" x2="197.13575" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.04925" x2="197.38975" y2="13.11275" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="13.11275" x2="173.76775" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.41875" y1="13.11275" x2="175.48225" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.11275" x2="176.05375" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.11275" x2="176.49825" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.11275" x2="176.62525" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="13.11275" x2="180.24475" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.11275" x2="180.43525" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.11275" x2="181.26075" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.54675" y1="13.11275" x2="184.37225" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="13.11275" x2="185.00725" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="13.11275" x2="185.51525" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="13.11275" x2="188.49975" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="13.11275" x2="188.94425" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="13.11275" x2="189.57925" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="13.11275" x2="192.94475" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="13.11275" x2="193.32575" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="13.11275" x2="194.84975" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.11275" x2="194.97675" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="13.11275" x2="197.26275" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.11275" x2="197.38975" y2="13.17625" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.51375" y1="13.17625" x2="173.83125" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.48225" y1="13.17625" x2="175.54575" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.17625" x2="176.05375" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.17625" x2="176.49825" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.17625" x2="176.62525" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="13.17625" x2="180.11775" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="13.17625" x2="180.37175" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="13.17625" x2="180.81625" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.17625" x2="181.26075" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="13.17625" x2="184.37225" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="13.17625" x2="184.94375" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="13.17625" x2="185.51525" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="13.17625" x2="188.49975" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="13.17625" x2="188.88075" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="13.17625" x2="189.57925" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="13.17625" x2="193.13525" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="13.17625" x2="193.32575" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="13.17625" x2="194.84975" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.17625" x2="194.97675" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.17625" x2="196.88175" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="13.17625" x2="197.19925" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="13.17625" x2="197.38975" y2="13.23975" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.57725" y1="13.23975" x2="173.83125" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.54575" y1="13.23975" x2="175.60925" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.23975" x2="176.05375" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.23975" x2="176.49825" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.23975" x2="176.62525" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="13.23975" x2="179.99075" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="13.23975" x2="180.37175" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.23975" x2="180.87975" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.23975" x2="181.26075" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="13.23975" x2="184.37225" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="13.23975" x2="184.88025" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="13.23975" x2="185.51525" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="13.23975" x2="188.81725" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="13.23975" x2="189.57925" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="13.23975" x2="193.19875" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="13.23975" x2="193.38925" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="13.23975" x2="194.84975" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.23975" x2="194.97675" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.23975" x2="196.81825" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.23975" x2="197.32625" y2="13.30325" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="13.30325" x2="173.89475" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="13.30325" x2="175.73625" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.30325" x2="176.05375" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.30325" x2="176.49825" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.30325" x2="176.62525" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="13.30325" x2="179.86375" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.99075" y1="13.30325" x2="180.37175" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.30325" x2="180.87975" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.30325" x2="181.26075" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="13.30325" x2="184.75325" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="13.30325" x2="185.45175" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.30325" x2="188.49975" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.56325" y1="13.30325" x2="188.75375" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="13.30325" x2="189.57925" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="13.30325" x2="193.19875" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.30325" x2="193.38925" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.02425" y1="13.30325" x2="194.78625" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.30325" x2="194.97675" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.30325" x2="196.75475" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.30325" x2="197.38975" y2="13.36675" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.64075" y1="13.36675" x2="173.89475" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="13.36675" x2="175.79975" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.36675" x2="176.05375" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.36675" x2="176.49825" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.36675" x2="176.62525" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="13.36675" x2="179.80025" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.86375" y1="13.36675" x2="180.37175" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.36675" x2="180.87975" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.36675" x2="181.26075" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.41975" y1="13.36675" x2="184.11825" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="13.36675" x2="184.68975" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="13.36675" x2="185.45175" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.36675" x2="188.49975" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.56325" y1="13.36675" x2="188.75375" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.45225" y1="13.36675" x2="189.51575" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.36675" x2="193.19875" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.36675" x2="193.38925" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.02425" y1="13.36675" x2="194.78625" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.36675" x2="194.97675" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="13.36675" x2="196.75475" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.36675" x2="197.38975" y2="13.43025" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.70425" y1="13.43025" x2="173.95825" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.43025" x2="176.05375" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.43025" x2="176.49825" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.43025" x2="176.62525" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="13.43025" x2="179.73675" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="13.43025" x2="180.37175" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="13.43025" x2="180.94325" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.43025" x2="181.26075" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.41975" y1="13.43025" x2="184.05475" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="13.43025" x2="184.56275" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="13.43025" x2="185.38825" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.43025" x2="188.69025" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.45225" y1="13.43025" x2="189.51575" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.43025" x2="193.19875" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.43025" x2="193.38925" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.96075" y1="13.43025" x2="194.78625" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.43025" x2="194.97675" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.43025" x2="196.75475" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.43025" x2="197.38975" y2="13.49375" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.76775" y1="13.49375" x2="174.02175" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.49375" x2="176.05375" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.49375" x2="176.49825" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.49375" x2="176.62525" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="13.49375" x2="179.60975" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="13.49375" x2="180.37175" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="13.49375" x2="180.94325" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="13.49375" x2="181.26075" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="13.49375" x2="183.92775" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="13.49375" x2="184.49925" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="13.49375" x2="185.32475" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.49375" x2="188.43625" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="13.49375" x2="188.62675" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.49375" x2="189.45225" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.49375" x2="193.19875" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.49375" x2="193.38925" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.89725" y1="13.49375" x2="194.78625" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.91325" y1="13.49375" x2="194.97675" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.49375" x2="196.75475" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.49375" x2="197.38975" y2="13.55725" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.83125" y1="13.55725" x2="174.08525" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.55725" x2="176.05375" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.55725" x2="176.49825" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.55725" x2="176.62525" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="13.55725" x2="179.54625" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="13.55725" x2="180.37175" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="13.55725" x2="180.94325" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="13.55725" x2="181.19725" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="13.55725" x2="183.80075" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.86425" y1="13.55725" x2="184.37225" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="13.55725" x2="185.26125" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.55725" x2="188.43625" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="13.55725" x2="188.62675" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.55725" x2="189.45225" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.55725" x2="193.26225" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.55725" x2="193.38925" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.83375" y1="13.55725" x2="194.78625" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="13.55725" x2="194.91325" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="13.55725" x2="196.81825" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="13.55725" x2="197.38975" y2="13.62075" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.89475" y1="13.62075" x2="174.14875" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.62075" x2="176.05375" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.62075" x2="176.49825" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.62075" x2="176.62525" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="13.62075" x2="179.48275" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="13.62075" x2="180.11775" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.62075" x2="180.37175" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.62075" x2="180.87975" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="13.62075" x2="181.13375" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="13.62075" x2="183.73725" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="13.62075" x2="184.24525" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="13.62075" x2="185.26125" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.62075" x2="188.56325" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.62075" x2="189.45225" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.62075" x2="193.26225" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.62075" x2="193.45275" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="13.62075" x2="194.72275" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="13.62075" x2="194.91325" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="13.62075" x2="196.88175" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="13.62075" x2="197.26275" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.62075" x2="197.38975" y2="13.68425" layer="94" grouprefs="LOGO"/>
<rectangle x1="173.95825" y1="13.68425" x2="174.21225" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.68425" x2="176.05375" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="13.68425" x2="176.49825" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.68425" x2="176.62525" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="13.68425" x2="179.99075" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.68425" x2="180.37175" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.68425" x2="180.87975" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="13.68425" x2="181.13375" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="13.68425" x2="183.67375" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="13.68425" x2="184.18175" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="13.68425" x2="185.19775" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.68425" x2="188.56325" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.68425" x2="189.45225" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.68425" x2="193.26225" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="13.68425" x2="193.45275" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="13.68425" x2="194.65925" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.78625" y1="13.68425" x2="194.84975" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="13.68425" x2="196.88175" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.68425" x2="197.38975" y2="13.74775" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.02175" y1="13.74775" x2="174.33925" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.92675" y1="13.74775" x2="176.05375" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="13.74775" x2="176.49825" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.74775" x2="176.62525" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="13.74775" x2="179.92725" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.74775" x2="180.37175" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="13.74775" x2="180.87975" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="13.74775" x2="181.07025" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="13.74775" x2="183.61025" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="13.74775" x2="184.05475" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.07075" y1="13.74775" x2="185.13425" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.74775" x2="188.37275" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.43625" y1="13.74775" x2="188.49975" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.74775" x2="189.45225" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.74775" x2="193.26225" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="13.74775" x2="193.45275" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="13.74775" x2="194.65925" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.78625" y1="13.74775" x2="194.84975" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="13.74775" x2="196.94525" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.74775" x2="197.38975" y2="13.81125" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.14875" y1="13.81125" x2="174.40275" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="13.81125" x2="175.99025" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="13.81125" x2="176.43475" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.81125" x2="176.62525" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="13.81125" x2="179.35575" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="13.81125" x2="179.86375" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.81125" x2="180.43525" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="13.81125" x2="180.81625" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="13.81125" x2="181.00675" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="13.81125" x2="183.54675" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="13.81125" x2="183.92775" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.00725" y1="13.81125" x2="185.07075" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.81125" x2="188.49975" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="13.81125" x2="189.45225" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.81125" x2="193.26225" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="13.81125" x2="193.45275" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="13.81125" x2="194.59575" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.72275" y1="13.81125" x2="194.78625" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="13.81125" x2="197.07225" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.81125" x2="197.38975" y2="13.87475" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.21225" y1="13.87475" x2="174.46625" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="13.87475" x2="175.92675" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="13.87475" x2="176.43475" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.87475" x2="176.62525" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="13.87475" x2="179.29225" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="13.87475" x2="179.80025" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.87475" x2="180.43525" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="13.87475" x2="180.81625" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="13.87475" x2="180.94325" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="13.87475" x2="183.80075" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="13.87475" x2="185.00725" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.87475" x2="188.49975" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.32525" y1="13.87475" x2="189.38875" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="13.87475" x2="193.26225" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="13.87475" x2="193.45275" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.57975" y1="13.87475" x2="194.59575" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.72275" y1="13.87475" x2="194.78625" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="13.87475" x2="197.19925" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="13.87475" x2="197.38975" y2="13.93825" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.27575" y1="13.93825" x2="174.59325" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="13.93825" x2="175.86325" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="13.93825" x2="176.43475" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="13.93825" x2="176.62525" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="13.93825" x2="179.22875" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="13.93825" x2="179.73675" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="13.93825" x2="180.43525" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="13.93825" x2="180.94325" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="13.93825" x2="183.73725" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="13.93825" x2="184.94375" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="13.93825" x2="185.38825" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="13.93825" x2="188.30925" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.37275" y1="13.93825" x2="188.43625" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.32525" y1="13.93825" x2="189.38875" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="13.93825" x2="192.11925" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="13.93825" x2="193.26225" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="13.93825" x2="194.53225" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="13.93825" x2="194.78625" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="13.93825" x2="197.38975" y2="14.00175" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.33925" y1="14.00175" x2="174.65675" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="14.00175" x2="175.73625" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="14.00175" x2="176.43475" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="14.00175" x2="176.62525" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="14.00175" x2="179.67325" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="14.00175" x2="180.43525" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.00175" x2="180.87975" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.03875" y1="14.00175" x2="183.67375" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="14.00175" x2="184.88025" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="14.00175" x2="185.45175" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="14.00175" x2="188.30925" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.37275" y1="14.00175" x2="188.43625" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.32525" y1="14.00175" x2="189.38875" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.00175" x2="192.05575" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.00175" x2="193.32575" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="14.00175" x2="194.53225" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.59575" y1="14.00175" x2="194.72275" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.78625" y1="14.00175" x2="194.84975" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="14.00175" x2="197.45325" y2="14.06525" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.46625" y1="14.06525" x2="174.78375" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="14.06525" x2="176.37125" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.06525" x2="176.56175" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="14.06525" x2="176.81575" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="14.06525" x2="179.16525" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="14.06525" x2="179.60975" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.06525" x2="180.43525" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.06525" x2="180.87975" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.03875" y1="14.06525" x2="183.61025" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="14.06525" x2="184.88025" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="14.06525" x2="185.51525" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="14.06525" x2="188.37275" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.32525" y1="14.06525" x2="189.38875" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.06525" x2="192.05575" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.06525" x2="194.46875" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.59575" y1="14.06525" x2="194.72275" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.06525" x2="194.91325" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="14.06525" x2="197.51675" y2="14.12875" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.52975" y1="14.12875" x2="174.84725" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.30775" y1="14.12875" x2="176.37125" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.12875" x2="176.56175" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="14.12875" x2="176.94275" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="14.12875" x2="179.10175" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="14.12875" x2="179.60975" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.12875" x2="180.43525" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.12875" x2="180.81625" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.12875" x2="183.48325" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="14.12875" x2="184.81675" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="14.12875" x2="185.45175" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="14.12875" x2="185.57875" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="14.12875" x2="188.24575" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.30925" y1="14.12875" x2="188.37275" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="14.12875" x2="189.32525" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.12875" x2="192.05575" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.12875" x2="194.46875" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.53225" y1="14.12875" x2="194.72275" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.12875" x2="194.91325" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="14.12875" x2="196.94525" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="14.12875" x2="197.51675" y2="14.19225" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.59325" y1="14.19225" x2="174.97425" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.24425" y1="14.19225" x2="176.30775" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.19225" x2="177.00625" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="14.19225" x2="179.54625" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.19225" x2="180.43525" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.19225" x2="180.81625" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.19225" x2="183.41975" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="14.19225" x2="184.75325" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="14.19225" x2="185.45175" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="14.19225" x2="185.64225" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="14.19225" x2="188.24575" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.30925" y1="14.19225" x2="188.37275" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="14.19225" x2="189.32525" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.19225" x2="192.05575" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.19225" x2="194.40525" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.53225" y1="14.19225" x2="194.59575" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.19225" x2="194.72275" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.19225" x2="194.91325" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.19225" x2="196.94525" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="14.19225" x2="197.51675" y2="14.25575" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.72025" y1="14.25575" x2="175.10125" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="14.25575" x2="176.62525" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="14.25575" x2="177.06975" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="14.25575" x2="179.48275" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.25575" x2="180.43525" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.25575" x2="180.81625" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.25575" x2="183.35625" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="14.25575" x2="184.68975" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.38825" y1="14.25575" x2="185.51525" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="14.25575" x2="185.64225" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="14.25575" x2="188.30925" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.19825" y1="14.25575" x2="189.26175" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.25575" x2="192.05575" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.25575" x2="194.34175" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.46875" y1="14.25575" x2="194.53225" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.25575" x2="194.72275" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.25575" x2="194.91325" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.25575" x2="196.94525" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="14.25575" x2="197.51675" y2="14.31925" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.84725" y1="14.31925" x2="175.22825" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.37125" y1="14.31925" x2="176.49825" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="14.31925" x2="176.87925" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.00625" y1="14.31925" x2="177.13325" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="14.31925" x2="179.41925" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.31925" x2="180.43525" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.31925" x2="180.81625" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.31925" x2="183.29275" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="14.31925" x2="184.62625" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="14.31925" x2="185.57875" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="14.31925" x2="185.70575" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="14.31925" x2="188.30925" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.19825" y1="14.31925" x2="189.26175" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.31925" x2="192.05575" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.31925" x2="194.34175" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.46875" y1="14.31925" x2="194.53225" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.31925" x2="194.72275" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.31925" x2="194.91325" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.31925" x2="196.94525" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="14.31925" x2="197.51675" y2="14.38275" layer="94" grouprefs="LOGO"/>
<rectangle x1="174.97425" y1="14.38275" x2="175.35525" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="14.38275" x2="176.81575" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.06975" y1="14.38275" x2="177.13325" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="14.38275" x2="179.41925" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.38275" x2="180.43525" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.38275" x2="180.81625" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.38275" x2="183.22925" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="14.38275" x2="184.56275" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="14.38275" x2="185.57875" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.70575" y1="14.38275" x2="185.76925" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="14.38275" x2="188.37275" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.19825" y1="14.38275" x2="189.26175" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.38275" x2="192.05575" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.38275" x2="194.27825" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="14.38275" x2="194.46875" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.38275" x2="194.72275" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.38275" x2="194.91325" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.38275" x2="196.94525" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.38275" x2="197.58025" y2="14.44625" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.10125" y1="14.44625" x2="175.48225" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.68875" y1="14.44625" x2="176.75225" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="14.44625" x2="177.19675" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.44625" x2="179.35575" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="14.44625" x2="180.43525" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.44625" x2="180.81625" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.44625" x2="183.22925" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="14.44625" x2="184.49925" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="14.44625" x2="185.64225" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.70575" y1="14.44625" x2="185.76925" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="14.44625" x2="188.11875" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.44625" x2="188.37275" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="14.44625" x2="189.19825" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.44625" x2="192.05575" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.44625" x2="194.27825" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.40525" y1="14.44625" x2="194.46875" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.44625" x2="194.72275" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.44625" x2="194.91325" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.44625" x2="196.94525" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.44625" x2="197.58025" y2="14.50975" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.22825" y1="14.50975" x2="175.54575" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="14.50975" x2="176.68875" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.13325" y1="14.50975" x2="177.19675" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.50975" x2="179.29225" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="14.50975" x2="180.43525" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.50975" x2="180.81625" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.50975" x2="183.22925" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="14.50975" x2="184.49925" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="14.50975" x2="185.70575" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="14.50975" x2="185.83275" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="14.50975" x2="188.05525" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.50975" x2="188.30925" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="14.50975" x2="189.19825" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.50975" x2="192.05575" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.50975" x2="194.21475" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.34175" y1="14.50975" x2="194.40525" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.50975" x2="194.72275" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.50975" x2="194.91325" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.50975" x2="196.94525" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.50975" x2="197.58025" y2="14.57325" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.35525" y1="14.57325" x2="175.67275" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.56175" y1="14.57325" x2="176.62525" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.06975" y1="14.57325" x2="177.13325" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.57325" x2="179.22875" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="14.57325" x2="180.37175" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.57325" x2="180.81625" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.57325" x2="183.16575" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="14.57325" x2="184.43575" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="14.57325" x2="185.57875" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="14.57325" x2="185.70575" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="14.57325" x2="185.83275" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.89625" y1="14.57325" x2="185.95975" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="14.57325" x2="187.99175" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.57325" x2="188.30925" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="14.57325" x2="189.13475" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.57325" x2="192.05575" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.57325" x2="194.21475" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="14.57325" x2="194.40525" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.57325" x2="194.72275" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.57325" x2="194.91325" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.57325" x2="196.94525" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.57325" x2="197.58025" y2="14.63675" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.48225" y1="14.63675" x2="175.73625" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.63675" x2="176.56175" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="177.00625" y1="14.63675" x2="177.06975" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.63675" x2="179.22875" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="14.63675" x2="180.37175" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.63675" x2="180.81625" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.63675" x2="183.16575" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="14.63675" x2="184.43575" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="14.63675" x2="185.57875" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="14.63675" x2="185.70575" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="14.63675" x2="185.95975" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="14.63675" x2="187.99175" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.63675" x2="188.30925" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="14.63675" x2="189.13475" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.63675" x2="192.05575" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.63675" x2="194.15125" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.27825" y1="14.63675" x2="194.34175" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.63675" x2="194.72275" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.63675" x2="194.91325" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.63675" x2="196.94525" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.63675" x2="197.58025" y2="14.70025" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.54575" y1="14.70025" x2="175.79975" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.70025" x2="176.56175" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.94275" y1="14.70025" x2="177.00625" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.70025" x2="179.16525" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="14.70025" x2="180.37175" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.70025" x2="180.81625" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="14.70025" x2="183.16575" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="14.70025" x2="184.37225" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="14.70025" x2="185.64225" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.83275" y1="14.70025" x2="186.02325" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="14.70025" x2="187.92825" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.70025" x2="188.30925" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="14.70025" x2="189.07125" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.70025" x2="192.05575" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.70025" x2="194.08775" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="14.70025" x2="194.34175" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.70025" x2="194.72275" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.70025" x2="194.91325" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.70025" x2="196.94525" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.70025" x2="197.58025" y2="14.76375" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.60925" y1="14.76375" x2="175.79975" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="14.76375" x2="176.56175" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.87925" y1="14.76375" x2="176.94275" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.76375" x2="179.10175" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="14.76375" x2="180.37175" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.76375" x2="180.81625" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.76375" x2="183.16575" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="14.76375" x2="184.18175" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="14.76375" x2="184.37225" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="14.76375" x2="185.64225" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="14.76375" x2="186.02325" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="14.76375" x2="187.92825" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="14.76375" x2="188.30925" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="14.76375" x2="189.00775" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.76375" x2="192.05575" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.76375" x2="194.08775" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.21475" y1="14.76375" x2="194.27825" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.76375" x2="194.72275" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.76375" x2="194.91325" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.76375" x2="197.00875" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="14.76375" x2="197.58025" y2="14.82725" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.67275" y1="14.82725" x2="175.86325" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="14.82725" x2="176.49825" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="14.82725" x2="176.94275" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.82725" x2="179.10175" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="14.82725" x2="180.37175" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.82725" x2="180.81625" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.82725" x2="183.16575" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.05475" y1="14.82725" x2="184.11825" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="14.82725" x2="184.37225" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="14.82725" x2="185.64225" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.70575" y1="14.82725" x2="185.89625" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.02325" y1="14.82725" x2="186.08675" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="14.82725" x2="187.92825" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="14.82725" x2="188.24575" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="14.82725" x2="189.00775" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.82725" x2="192.05575" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.82725" x2="194.02425" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="14.82725" x2="194.21475" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.82725" x2="194.72275" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.82725" x2="194.91325" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.82725" x2="197.00875" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="14.82725" x2="197.64375" y2="14.89075" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.73625" y1="14.89075" x2="175.92675" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="14.89075" x2="176.49825" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="14.89075" x2="176.94275" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.89075" x2="179.10175" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="14.89075" x2="180.37175" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.89075" x2="180.81625" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.89075" x2="183.16575" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="14.89075" x2="184.43575" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="14.89075" x2="185.76925" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.02325" y1="14.89075" x2="186.08675" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="14.89075" x2="187.92825" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="14.89075" x2="188.24575" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="14.89075" x2="188.94425" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.89075" x2="192.05575" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.89075" x2="194.02425" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.15125" y1="14.89075" x2="194.21475" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.89075" x2="194.72275" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.89075" x2="194.91325" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.89075" x2="197.00875" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="14.89075" x2="197.64375" y2="14.95425" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="14.95425" x2="175.99025" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.43475" y1="14.95425" x2="176.49825" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="14.95425" x2="176.94275" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.78425" y1="14.95425" x2="179.10175" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.99075" y1="14.95425" x2="180.05425" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="14.95425" x2="180.37175" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="14.95425" x2="180.81625" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="14.95425" x2="183.22925" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="14.95425" x2="184.43575" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="14.95425" x2="185.64225" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.08675" y1="14.95425" x2="186.15025" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="14.95425" x2="187.92825" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="14.95425" x2="188.18225" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="14.95425" x2="188.94425" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="14.95425" x2="192.05575" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="14.95425" x2="193.96075" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="14.95425" x2="194.15125" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="14.95425" x2="194.72275" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="14.95425" x2="194.91325" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="14.95425" x2="197.00875" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="14.95425" x2="197.64375" y2="15.01775" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.79975" y1="15.01775" x2="176.05375" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.49825" y1="15.01775" x2="176.56175" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.81575" y1="15.01775" x2="176.87925" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="15.01775" x2="179.16525" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.01775" x2="180.37175" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.01775" x2="180.81625" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.97525" y1="15.01775" x2="183.22925" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="15.01775" x2="184.43575" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="15.01775" x2="185.51525" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.08675" y1="15.01775" x2="186.15025" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.01775" x2="187.92825" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="15.01775" x2="188.11875" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.01775" x2="188.88075" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="15.01775" x2="192.05575" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.01775" x2="193.96075" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.08775" y1="15.01775" x2="194.15125" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="15.01775" x2="194.72275" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="15.01775" x2="194.91325" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.01775" x2="197.00875" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="15.01775" x2="197.64375" y2="15.08125" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.86325" y1="15.08125" x2="176.11725" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="15.08125" x2="176.87925" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.84775" y1="15.08125" x2="179.16525" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.08125" x2="180.37175" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.08125" x2="180.81625" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="15.08125" x2="183.29275" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="15.08125" x2="185.26125" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.02325" y1="15.08125" x2="186.08675" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.08125" x2="187.86475" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="15.08125" x2="188.05525" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.08125" x2="188.88075" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="15.08125" x2="192.05575" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.08125" x2="193.89725" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.02425" y1="15.08125" x2="194.08775" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="15.08125" x2="194.72275" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="15.08125" x2="194.91325" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.08125" x2="197.00875" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="15.08125" x2="197.64375" y2="15.14475" layer="94" grouprefs="LOGO"/>
<rectangle x1="175.99025" y1="15.14475" x2="176.18075" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.75225" y1="15.14475" x2="176.81575" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="15.14475" x2="179.16525" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.14475" x2="180.37175" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.14475" x2="180.81625" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.84825" y1="15.14475" x2="183.35625" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="15.14475" x2="184.88025" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.95975" y1="15.14475" x2="186.02325" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.14475" x2="187.86475" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="15.14475" x2="188.05525" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="15.14475" x2="188.88075" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="15.14475" x2="192.05575" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.14475" x2="193.83375" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.96075" y1="15.14475" x2="194.08775" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="15.14475" x2="194.72275" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="15.14475" x2="194.91325" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.14475" x2="197.00875" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="15.14475" x2="197.64375" y2="15.20825" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.11725" y1="15.20825" x2="176.24425" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="176.62525" y1="15.20825" x2="176.75225" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.91125" y1="15.20825" x2="179.22875" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.20825" x2="180.37175" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.20825" x2="180.81625" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.84825" y1="15.20825" x2="183.03875" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.20825" x2="183.41975" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="15.20825" x2="184.62625" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.89625" y1="15.20825" x2="185.95975" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.20825" x2="187.86475" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="15.20825" x2="187.99175" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="15.20825" x2="188.88075" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.99225" y1="15.20825" x2="192.05575" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.20825" x2="192.88125" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="15.20825" x2="193.83375" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.96075" y1="15.20825" x2="194.02425" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="15.20825" x2="194.72275" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="15.20825" x2="194.91325" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.20825" x2="197.00875" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="15.20825" x2="197.64375" y2="15.27175" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="15.27175" x2="179.22875" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.27175" x2="180.37175" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.27175" x2="180.87975" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="15.27175" x2="183.03875" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.27175" x2="183.41975" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.05475" y1="15.27175" x2="184.49925" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.13425" y1="15.27175" x2="185.19775" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.27175" x2="187.80125" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="15.27175" x2="187.99175" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="15.27175" x2="188.75375" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.27175" x2="188.88075" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.05575" y1="15.27175" x2="192.11925" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.27175" x2="192.81775" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="15.27175" x2="193.77025" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.89725" y1="15.27175" x2="194.02425" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.65925" y1="15.27175" x2="194.72275" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.84975" y1="15.27175" x2="194.91325" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.27175" x2="197.00875" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="15.27175" x2="197.70725" y2="15.33525" layer="94" grouprefs="LOGO"/>
<rectangle x1="178.97475" y1="15.33525" x2="179.22875" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.33525" x2="180.37175" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.33525" x2="180.87975" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="15.33525" x2="182.97525" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.33525" x2="183.48325" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="15.33525" x2="184.37225" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.00725" y1="15.33525" x2="185.13425" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="15.33525" x2="187.80125" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="15.33525" x2="187.99175" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="15.33525" x2="188.69025" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.33525" x2="188.88075" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="15.33525" x2="192.18275" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.33525" x2="192.81775" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="15.33525" x2="193.77025" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.89725" y1="15.33525" x2="193.96075" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.78625" y1="15.33525" x2="194.91325" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.33525" x2="197.00875" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="15.33525" x2="197.70725" y2="15.39875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="15.39875" x2="179.29225" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.39875" x2="180.37175" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="15.39875" x2="180.94325" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="15.39875" x2="182.97525" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.39875" x2="183.48325" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.86425" y1="15.39875" x2="184.24525" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="15.39875" x2="185.07075" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="15.39875" x2="187.73775" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.39875" x2="187.99175" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.39875" x2="188.88075" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.39875" x2="192.81775" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="15.39875" x2="193.70675" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.83375" y1="15.39875" x2="193.96075" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="194.78625" y1="15.39875" x2="194.84975" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.39875" x2="197.00875" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="15.39875" x2="197.70725" y2="15.46225" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.03825" y1="15.46225" x2="179.29225" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.46225" x2="180.37175" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="15.46225" x2="180.94325" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="15.46225" x2="182.91175" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.46225" x2="183.48325" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="15.46225" x2="184.18175" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="15.46225" x2="184.94375" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="15.46225" x2="187.67425" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.46225" x2="187.92825" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.46225" x2="188.88075" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.46225" x2="192.75425" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="15.46225" x2="193.70675" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.83375" y1="15.46225" x2="193.89725" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.46225" x2="197.00875" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="15.46225" x2="197.70725" y2="15.52575" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="15.52575" x2="179.35575" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.11775" y1="15.52575" x2="180.37175" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="15.52575" x2="181.00675" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="15.52575" x2="182.91175" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.52575" x2="183.48325" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="15.52575" x2="184.11825" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="15.52575" x2="184.88025" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="15.52575" x2="187.67425" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.52575" x2="187.92825" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.52575" x2="188.88075" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.52575" x2="192.75425" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="15.52575" x2="193.64325" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="15.52575" x2="193.83375" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.52575" x2="197.00875" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="15.52575" x2="197.70725" y2="15.58925" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.10175" y1="15.58925" x2="179.35575" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="15.58925" x2="180.43525" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="15.58925" x2="181.00675" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="15.58925" x2="182.91175" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.58925" x2="183.48325" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="15.58925" x2="184.05475" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="15.58925" x2="184.81675" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="15.58925" x2="187.61075" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.58925" x2="187.92825" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.58925" x2="188.88075" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.58925" x2="192.75425" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="15.58925" x2="193.57975" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="15.58925" x2="193.83375" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="15.58925" x2="197.00875" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="15.58925" x2="197.64375" y2="15.65275" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="15.65275" x2="179.35575" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="15.65275" x2="180.43525" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="15.65275" x2="181.07025" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="15.65275" x2="182.91175" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.65275" x2="183.48325" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.65275" x2="183.99125" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="15.65275" x2="184.75325" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="15.65275" x2="187.61075" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.65275" x2="187.92825" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.65275" x2="188.88075" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.65275" x2="192.75425" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="15.65275" x2="193.26225" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="15.65275" x2="193.77025" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="15.65275" x2="197.00875" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="15.65275" x2="197.58025" y2="15.71625" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.16525" y1="15.71625" x2="179.41925" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="15.71625" x2="180.49875" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="15.71625" x2="181.13375" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="15.71625" x2="182.91175" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.71625" x2="183.22925" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="15.71625" x2="183.48325" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.71625" x2="183.99125" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="15.71625" x2="184.62625" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="15.71625" x2="187.54725" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.71625" x2="187.92825" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.71625" x2="188.88075" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="15.71625" x2="192.75425" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="15.71625" x2="193.26225" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="15.71625" x2="193.77025" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="15.71625" x2="197.00875" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="15.71625" x2="197.58025" y2="15.77975" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="15.77975" x2="179.41925" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="15.77975" x2="180.49875" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="15.77975" x2="181.19725" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="15.77975" x2="182.84825" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.77975" x2="183.22925" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="15.77975" x2="183.48325" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.77975" x2="183.99125" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="15.77975" x2="184.62625" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="15.77975" x2="187.54725" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.77975" x2="187.92825" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.77975" x2="188.88075" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.16675" y1="15.77975" x2="191.35725" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.77975" x2="192.75425" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="15.77975" x2="193.57975" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="15.77975" x2="197.38975" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="15.77975" x2="197.58025" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="15.77975" x2="197.89775" y2="15.84325" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.22875" y1="15.84325" x2="179.48275" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.30825" y1="15.84325" x2="180.56225" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="15.84325" x2="181.26075" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="15.84325" x2="182.84825" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.84325" x2="183.22925" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="15.84325" x2="183.48325" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.84325" x2="183.99125" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.24525" y1="15.84325" x2="184.30875" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="15.84325" x2="184.62625" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="15.84325" x2="187.54725" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.84325" x2="187.92825" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.84325" x2="188.88075" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.10325" y1="15.84325" x2="191.35725" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.84325" x2="192.81775" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="15.84325" x2="193.57975" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="15.84325" x2="193.70675" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="15.84325" x2="197.38975" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="15.84325" x2="197.58025" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="15.84325" x2="197.96125" y2="15.90675" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="15.90675" x2="179.48275" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="15.90675" x2="180.56225" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="15.90675" x2="181.26075" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="15.90675" x2="182.84825" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="15.90675" x2="183.48325" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.90675" x2="183.99125" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.24525" y1="15.90675" x2="184.30875" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="15.90675" x2="184.62625" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="15.90675" x2="187.54725" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.90675" x2="187.92825" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.90675" x2="188.88075" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.03975" y1="15.90675" x2="191.23025" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.29375" y1="15.90675" x2="191.35725" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.90675" x2="192.88125" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="15.90675" x2="193.26225" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="15.90675" x2="193.70675" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.88175" y1="15.90675" x2="197.07225" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="15.90675" x2="197.38975" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="15.90675" x2="197.58025" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="15.90675" x2="197.83425" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="15.90675" x2="197.96125" y2="15.97025" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.29225" y1="15.97025" x2="179.48275" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="15.97025" x2="180.62575" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.26075" y1="15.97025" x2="181.32425" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="15.97025" x2="182.84825" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="15.97025" x2="183.16575" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="15.97025" x2="183.48325" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="15.97025" x2="183.99125" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="15.97025" x2="184.37225" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="15.97025" x2="184.62625" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="15.97025" x2="187.48375" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="15.97025" x2="187.92825" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="15.97025" x2="188.88075" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.97625" y1="15.97025" x2="191.29375" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="15.97025" x2="192.81775" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="15.97025" x2="193.19875" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="15.97025" x2="193.70675" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="15.97025" x2="197.00875" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="15.97025" x2="197.77075" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="15.97025" x2="197.96125" y2="16.03375" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="16.03375" x2="179.54625" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="16.03375" x2="180.62575" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.32425" y1="16.03375" x2="181.38775" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="16.03375" x2="182.84825" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="16.03375" x2="183.48325" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.03375" x2="183.99125" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="16.03375" x2="184.37225" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="16.03375" x2="184.62625" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.03375" x2="187.48375" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.03375" x2="187.92825" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.03375" x2="188.88075" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.91275" y1="16.03375" x2="191.16675" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="16.03375" x2="192.81775" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="16.03375" x2="193.13525" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="16.03375" x2="193.70675" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="16.03375" x2="196.94525" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="16.03375" x2="197.70725" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.03375" x2="197.89775" y2="16.09725" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="16.09725" x2="179.54625" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="16.09725" x2="180.68925" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.32425" y1="16.09725" x2="181.38775" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="16.09725" x2="182.84825" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="16.09725" x2="183.48325" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.09725" x2="183.99125" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.09725" x2="184.43575" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="16.09725" x2="184.62625" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.09725" x2="187.48375" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.09725" x2="187.92825" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.09725" x2="188.88075" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.78575" y1="16.09725" x2="190.97625" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="191.03975" y1="16.09725" x2="191.10325" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="16.09725" x2="192.75425" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="16.09725" x2="193.32575" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="16.09725" x2="193.77025" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="16.09725" x2="196.94525" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.09725" x2="197.89775" y2="16.16075" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.35575" y1="16.16075" x2="179.60975" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.49875" y1="16.16075" x2="180.68925" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.32425" y1="16.16075" x2="181.38775" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="16.16075" x2="182.84825" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="16.16075" x2="183.48325" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.16075" x2="183.99125" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.16075" x2="184.62625" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.16075" x2="187.48375" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.16075" x2="187.92825" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.16075" x2="188.88075" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.72225" y1="16.16075" x2="190.91275" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.97625" y1="16.16075" x2="191.03975" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="16.16075" x2="192.75425" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="16.16075" x2="193.13525" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="16.16075" x2="193.77025" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="16.16075" x2="196.94525" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.16075" x2="197.89775" y2="16.22425" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="16.22425" x2="179.60975" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.56225" y1="16.22425" x2="180.75275" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="16.22425" x2="181.45125" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.65775" y1="16.22425" x2="182.91175" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="16.22425" x2="183.48325" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.22425" x2="183.99125" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.22425" x2="184.43575" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="16.22425" x2="184.68975" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.22425" x2="187.48375" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.22425" x2="187.92825" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.22425" x2="188.88075" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.65875" y1="16.22425" x2="190.97625" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="16.22425" x2="192.75425" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="16.22425" x2="193.13525" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="16.22425" x2="193.77025" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="16.22425" x2="197.00875" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.22425" x2="197.89775" y2="16.28775" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.41925" y1="16.28775" x2="179.67325" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.56225" y1="16.28775" x2="180.75275" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="16.28775" x2="181.45125" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.72125" y1="16.28775" x2="182.97525" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="16.28775" x2="183.48325" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.28775" x2="183.99125" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.28775" x2="184.43575" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="16.28775" x2="184.68975" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.28775" x2="187.48375" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.28775" x2="187.92825" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.28775" x2="188.88075" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.59525" y1="16.28775" x2="190.84925" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="16.28775" x2="192.69075" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="16.28775" x2="193.51625" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="16.28775" x2="193.77025" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="16.28775" x2="196.81825" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.88175" y1="16.28775" x2="197.07225" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.28775" x2="197.89775" y2="16.35125" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="16.35125" x2="179.67325" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.62575" y1="16.35125" x2="180.81625" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="16.35125" x2="181.45125" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.78475" y1="16.35125" x2="183.03875" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="16.35125" x2="183.48325" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.35125" x2="183.99125" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.35125" x2="184.43575" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="16.35125" x2="184.75325" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="16.35125" x2="187.48375" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="16.35125" x2="187.86475" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.35125" x2="188.88075" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.46825" y1="16.35125" x2="190.65875" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.72225" y1="16.35125" x2="190.78575" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="16.35125" x2="192.69075" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="16.35125" x2="193.57975" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="16.35125" x2="193.77025" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="16.35125" x2="196.88175" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.94525" y1="16.35125" x2="197.00875" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.35125" x2="197.89775" y2="16.41475" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.48275" y1="16.41475" x2="179.67325" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="16.41475" x2="180.81625" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="16.41475" x2="181.45125" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.84825" y1="16.41475" x2="183.10225" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="16.41475" x2="183.54675" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="16.41475" x2="183.99125" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.41475" x2="184.43575" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="16.41475" x2="184.75325" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.41475" x2="187.54725" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.41475" x2="187.92825" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.41475" x2="188.88075" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.40475" y1="16.41475" x2="190.59525" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.65875" y1="16.41475" x2="190.72225" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="16.41475" x2="192.62725" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="16.41475" x2="193.57975" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="16.41475" x2="193.77025" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="16.41475" x2="196.94525" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="16.41475" x2="197.07225" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.41475" x2="197.89775" y2="16.47825" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="16.47825" x2="179.73675" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="16.47825" x2="180.87975" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="16.47825" x2="181.45125" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.91175" y1="16.47825" x2="183.16575" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="16.47825" x2="183.54675" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="16.47825" x2="183.99125" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.47825" x2="184.43575" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.47825" x2="184.81675" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.47825" x2="187.54725" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="16.47825" x2="187.99175" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.47825" x2="188.88075" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.34125" y1="16.47825" x2="190.65875" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="16.47825" x2="192.62725" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="16.47825" x2="193.45275" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="16.47825" x2="193.77025" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="16.47825" x2="197.00875" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.47825" x2="197.89775" y2="16.54175" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.54625" y1="16.54175" x2="179.73675" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="16.54175" x2="180.87975" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="16.54175" x2="181.51475" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.03875" y1="16.54175" x2="183.22925" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="16.54175" x2="183.54675" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="16.54175" x2="183.99125" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.54175" x2="184.43575" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.54175" x2="184.81675" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.54175" x2="187.54725" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="16.54175" x2="187.99175" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.54175" x2="189.13475" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.27775" y1="16.54175" x2="190.53175" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="16.54175" x2="192.62725" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="16.54175" x2="193.38925" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="16.54175" x2="193.77025" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.88175" y1="16.54175" x2="197.07225" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.54175" x2="197.89775" y2="16.60525" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.60975" y1="16.60525" x2="179.80025" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="16.60525" x2="180.94325" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="16.60525" x2="181.51475" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.10225" y1="16.60525" x2="183.29275" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="16.60525" x2="183.61025" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="16.60525" x2="183.99125" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="16.60525" x2="184.43575" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.60525" x2="184.81675" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.60525" x2="187.54725" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="16.60525" x2="188.05525" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="16.60525" x2="189.19825" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.15075" y1="16.60525" x2="190.34125" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.40475" y1="16.60525" x2="190.46825" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="16.60525" x2="192.56375" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="16.60525" x2="193.32575" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.64325" y1="16.60525" x2="193.77025" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="16.60525" x2="197.19925" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.60525" x2="197.89775" y2="16.66875" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="16.66875" x2="179.80025" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.81625" y1="16.66875" x2="180.94325" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="16.66875" x2="181.51475" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.16575" y1="16.66875" x2="183.61025" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="16.66875" x2="183.99125" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="16.66875" x2="184.49925" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.66875" x2="184.81675" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.66875" x2="187.54725" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="16.66875" x2="188.11875" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="16.66875" x2="189.26175" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.08725" y1="16.66875" x2="190.40475" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="16.66875" x2="192.56375" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="16.66875" x2="193.26225" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.57975" y1="16.66875" x2="193.77025" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="16.66875" x2="197.19925" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.66875" x2="197.89775" y2="16.73225" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.67325" y1="16.73225" x2="179.86375" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="16.73225" x2="181.00675" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.51475" y1="16.73225" x2="181.57825" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.22925" y1="16.73225" x2="183.61025" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="16.73225" x2="183.99125" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="16.73225" x2="184.49925" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.73225" x2="184.81675" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.73225" x2="187.54725" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="16.73225" x2="188.11875" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="16.73225" x2="189.00775" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="16.73225" x2="189.32525" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.02375" y1="16.73225" x2="190.27775" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="16.73225" x2="192.50025" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="16.73225" x2="193.26225" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.57975" y1="16.73225" x2="193.77025" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="16.73225" x2="197.19925" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.73225" x2="197.89775" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="16.73225" x2="198.34225" y2="16.79575" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.73675" y1="16.79575" x2="179.92725" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="16.79575" x2="181.00675" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.57825" y1="16.79575" x2="181.64175" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.29275" y1="16.79575" x2="183.99125" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="16.79575" x2="184.56275" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.75325" y1="16.79575" x2="184.81675" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.79575" x2="187.54725" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="16.79575" x2="188.18225" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="16.79575" x2="188.94425" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="16.79575" x2="189.45225" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.96025" y1="16.79575" x2="190.21425" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="16.79575" x2="192.50025" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="16.79575" x2="193.38925" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="16.79575" x2="193.77025" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="16.79575" x2="197.45325" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.79575" x2="197.89775" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="16.79575" x2="198.53275" y2="16.85925" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.80025" y1="16.85925" x2="179.99075" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="16.85925" x2="181.07025" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="16.85925" x2="181.70525" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.89575" y1="16.85925" x2="182.34025" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.35625" y1="16.85925" x2="183.99125" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="16.85925" x2="184.62625" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="16.85925" x2="184.75325" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="16.85925" x2="187.54725" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="16.85925" x2="188.18225" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.85925" x2="188.94425" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="16.85925" x2="189.57925" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.83325" y1="16.85925" x2="190.02375" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="190.08725" y1="16.85925" x2="190.15075" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="16.85925" x2="192.50025" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="16.85925" x2="193.26225" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="16.85925" x2="193.77025" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="16.85925" x2="197.64375" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.85925" x2="197.89775" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="16.85925" x2="198.27875" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.53275" y1="16.85925" x2="198.59625" y2="16.92275" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.86375" y1="16.92275" x2="180.05425" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="16.92275" x2="181.07025" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="16.92275" x2="182.34025" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.40375" y1="16.92275" x2="182.46725" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.41975" y1="16.92275" x2="183.99125" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="16.92275" x2="184.62625" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.68975" y1="16.92275" x2="184.75325" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="16.92275" x2="187.61075" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.11875" y1="16.92275" x2="188.18225" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="16.92275" x2="188.94425" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="16.92275" x2="190.08725" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.24625" y1="16.92275" x2="192.43675" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="16.92275" x2="193.26225" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="16.92275" x2="193.77025" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="16.92275" x2="197.70725" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.92275" x2="197.89775" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="16.92275" x2="198.21525" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.53275" y1="16.92275" x2="198.59625" y2="16.98625" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.92725" y1="16.98625" x2="180.18125" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.94325" y1="16.98625" x2="181.13375" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.70525" y1="16.98625" x2="182.27675" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="16.98625" x2="182.53075" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="16.98625" x2="184.05475" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="16.98625" x2="184.75325" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="16.98625" x2="187.61075" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="16.98625" x2="188.24575" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="16.98625" x2="189.00775" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.57925" y1="16.98625" x2="190.02375" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.24625" y1="16.98625" x2="192.43675" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="16.98625" x2="193.38925" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="16.98625" x2="193.77025" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="16.98625" x2="197.70725" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="16.98625" x2="197.89775" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="16.98625" x2="198.27875" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.53275" y1="16.98625" x2="198.59625" y2="17.04975" layer="94" grouprefs="LOGO"/>
<rectangle x1="179.99075" y1="17.04975" x2="180.24475" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="17.04975" x2="181.19725" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.70525" y1="17.04975" x2="182.21325" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="17.04975" x2="182.53075" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="17.04975" x2="184.05475" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="17.04975" x2="184.68975" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="17.04975" x2="187.61075" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.04975" x2="188.24575" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="17.04975" x2="189.00775" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="17.04975" x2="189.13475" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.19825" y1="17.04975" x2="189.26175" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.64275" y1="17.04975" x2="190.02375" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.04975" x2="192.37325" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="17.04975" x2="193.32575" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.04975" x2="193.77025" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="17.04975" x2="197.38975" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="17.04975" x2="198.02475" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="17.04975" x2="198.27875" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.53275" y1="17.04975" x2="198.59625" y2="17.11325" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.05425" y1="17.11325" x2="180.37175" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.70525" y1="17.11325" x2="182.08625" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.53075" y1="17.11325" x2="182.59425" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.48325" y1="17.11325" x2="184.05475" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.56275" y1="17.11325" x2="184.68975" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="17.11325" x2="187.61075" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.11325" x2="188.24575" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="17.11325" x2="189.07125" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="17.11325" x2="189.45225" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.51575" y1="17.11325" x2="189.76975" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.96025" y1="17.11325" x2="190.02375" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.11325" x2="192.37325" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.11325" x2="193.32575" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.11325" x2="193.77025" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="17.11325" x2="197.32625" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="17.11325" x2="197.96125" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.27875" y1="17.11325" x2="198.40575" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="17.11325" x2="198.53275" y2="17.17675" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.18125" y1="17.17675" x2="180.43525" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="17.17675" x2="182.02275" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="17.17675" x2="182.59425" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.54675" y1="17.17675" x2="184.05475" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="17.17675" x2="184.56275" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.62625" y1="17.17675" x2="184.68975" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="17.17675" x2="187.67425" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.17675" x2="188.24575" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="17.17675" x2="189.00775" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.45225" y1="17.17675" x2="189.57925" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.89675" y1="17.17675" x2="189.96025" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.17675" x2="192.50025" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.17675" x2="193.26225" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.17675" x2="193.77025" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="17.17675" x2="197.32625" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="17.17675" x2="197.96125" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.27875" y1="17.17675" x2="198.34225" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="17.17675" x2="198.59625" y2="17.24025" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.24475" y1="17.24025" x2="180.56225" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="17.24025" x2="182.02275" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="17.24025" x2="182.53075" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.61025" y1="17.24025" x2="183.99125" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="17.24025" x2="184.43575" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.00725" y1="17.24025" x2="185.26125" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="17.24025" x2="187.67425" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.24025" x2="188.24575" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="17.24025" x2="189.00775" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.38875" y1="17.24025" x2="189.57925" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.83325" y1="17.24025" x2="189.89675" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.24025" x2="192.43675" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.24025" x2="193.26225" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.24025" x2="193.77025" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="17.24025" x2="197.32625" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="17.24025" x2="197.96125" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="17.24025" x2="198.27875" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.65975" y1="17.24025" x2="198.72325" y2="17.30375" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.37175" y1="17.30375" x2="180.68925" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.64175" y1="17.30375" x2="181.95925" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="17.30375" x2="182.53075" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.67375" y1="17.30375" x2="183.99125" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.30375" x2="185.38825" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.42025" y1="17.30375" x2="187.67425" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.30375" x2="188.24575" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="17.30375" x2="188.94425" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.32525" y1="17.30375" x2="189.64275" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.70625" y1="17.30375" x2="189.83325" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.30375" x2="192.37325" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.30375" x2="193.19875" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.30375" x2="193.77025" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.30375" x2="197.32625" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="17.30375" x2="198.27875" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.78675" y1="17.30375" x2="198.85025" y2="17.36725" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.43525" y1="17.36725" x2="180.75275" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.57825" y1="17.36725" x2="181.95925" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.46725" y1="17.36725" x2="182.53075" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.73725" y1="17.36725" x2="183.99125" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.36725" x2="185.45175" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="17.36725" x2="187.73775" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.36725" x2="188.24575" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="17.36725" x2="188.94425" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.26175" y1="17.36725" x2="189.51575" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.36725" x2="192.50025" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.36725" x2="193.13525" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.36725" x2="193.77025" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="17.36725" x2="197.45325" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="17.36725" x2="197.89775" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="17.36725" x2="198.27875" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.97725" y1="17.36725" x2="199.04075" y2="17.43075" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.56225" y1="17.43075" x2="180.87975" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.51475" y1="17.43075" x2="181.95925" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.40375" y1="17.43075" x2="182.46725" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.80075" y1="17.43075" x2="184.05475" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.43575" y1="17.43075" x2="184.62625" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.43075" x2="185.51525" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="17.43075" x2="187.80125" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.18225" y1="17.43075" x2="188.24575" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="17.43075" x2="188.88075" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="17.43075" x2="189.45225" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.43075" x2="192.37325" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.75425" y1="17.43075" x2="193.13525" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.43075" x2="193.77025" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="17.43075" x2="197.38975" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="17.43075" x2="197.64375" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="17.43075" x2="197.83425" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="17.43075" x2="198.46925" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.16775" y1="17.43075" x2="199.23125" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.67575" y1="17.43075" x2="199.86625" y2="17.49425" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.68925" y1="17.49425" x2="181.00675" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.38775" y1="17.49425" x2="181.95925" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.34025" y1="17.49425" x2="182.40375" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.86425" y1="17.49425" x2="184.11825" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.30875" y1="17.49425" x2="184.62625" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="17.49425" x2="185.32475" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.49425" x2="185.51525" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="17.49425" x2="187.80125" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="17.49425" x2="188.88075" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="17.49425" x2="189.32525" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.49425" x2="192.50025" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.75425" y1="17.49425" x2="193.07175" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.49425" x2="193.77025" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="17.49425" x2="197.38975" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="17.49425" x2="197.83425" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="17.49425" x2="198.27875" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.53275" y1="17.49425" x2="198.65975" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="17.49425" x2="199.35825" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="17.49425" x2="199.73925" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.80275" y1="17.49425" x2="199.99325" y2="17.55775" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.75275" y1="17.55775" x2="181.70525" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.76875" y1="17.55775" x2="181.95925" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.34025" y1="17.55775" x2="182.40375" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.92775" y1="17.55775" x2="184.62625" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="17.55775" x2="185.26125" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.55775" x2="185.51525" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="17.55775" x2="187.80125" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="17.55775" x2="188.88075" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="17.55775" x2="189.26175" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.11925" y1="17.55775" x2="192.37325" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.55775" x2="193.07175" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.55775" x2="193.77025" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="17.55775" x2="196.94525" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.55775" x2="197.38975" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="17.55775" x2="197.77075" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="17.55775" x2="198.15175" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="17.55775" x2="198.27875" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.78675" y1="17.55775" x2="198.91375" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.42175" y1="17.55775" x2="199.67575" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.92975" y1="17.55775" x2="199.99325" y2="17.62125" layer="94" grouprefs="LOGO"/>
<rectangle x1="180.87975" y1="17.62125" x2="181.64175" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.83225" y1="17.62125" x2="181.95925" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.27675" y1="17.62125" x2="182.34025" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="183.99125" y1="17.62125" x2="184.62625" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="17.62125" x2="185.19775" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.62125" x2="185.51525" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="17.62125" x2="187.80125" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="17.62125" x2="188.81725" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="17.62125" x2="189.19825" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.62125" x2="192.37325" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.62125" x2="193.00825" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.62125" x2="193.77025" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.50075" y1="17.62125" x2="196.94525" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.62125" x2="197.38975" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="17.62125" x2="197.77075" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="17.62125" x2="198.27875" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.97725" y1="17.62125" x2="199.04075" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.42175" y1="17.62125" x2="199.48525" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="17.62125" x2="199.73925" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.92975" y1="17.62125" x2="200.05675" y2="17.68475" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.00675" y1="17.68475" x2="181.64175" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.83225" y1="17.68475" x2="181.95925" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.08625" y1="17.68475" x2="182.34025" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.05475" y1="17.68475" x2="184.68975" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="17.68475" x2="185.19775" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="17.68475" x2="185.57875" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="17.68475" x2="187.80125" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.75375" y1="17.68475" x2="189.13475" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.68475" x2="192.43675" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="17.68475" x2="193.00825" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.68475" x2="193.77025" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="17.68475" x2="196.94525" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.68475" x2="197.38975" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="17.68475" x2="197.77075" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="17.68475" x2="198.08825" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="17.68475" x2="198.27875" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.35825" y1="17.68475" x2="199.48525" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.67575" y1="17.68475" x2="199.73925" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.92975" y1="17.68475" x2="200.12025" y2="17.74825" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.07025" y1="17.74825" x2="181.64175" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.83225" y1="17.74825" x2="182.02275" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.08625" y1="17.74825" x2="182.27675" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="17.74825" x2="184.68975" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.81675" y1="17.74825" x2="185.13425" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="17.74825" x2="185.57875" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="17.74825" x2="187.86475" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="17.74825" x2="189.00775" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.18275" y1="17.74825" x2="192.43675" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="17.74825" x2="193.00825" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.74825" x2="193.77025" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="17.74825" x2="197.07225" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.74825" x2="197.38975" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="17.74825" x2="197.70725" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="17.74825" x2="198.15175" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.34225" y1="17.74825" x2="198.40575" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.16775" y1="17.74825" x2="199.54875" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="17.74825" x2="199.67575" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="17.74825" x2="200.05675" y2="17.81175" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="17.81175" x2="181.64175" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.76875" y1="17.81175" x2="182.08625" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="182.21325" y1="17.81175" x2="182.27675" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="17.81175" x2="184.81675" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.81175" x2="185.07075" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.81175" x2="185.51525" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="17.81175" x2="187.86475" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="17.81175" x2="188.94425" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.24625" y1="17.81175" x2="192.50025" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="17.81175" x2="192.94475" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.81175" x2="193.77025" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="17.81175" x2="196.62775" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="17.81175" x2="197.13575" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="17.81175" x2="197.32625" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="17.81175" x2="197.70725" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="17.81175" x2="198.02475" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="17.81175" x2="198.21525" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="17.81175" x2="198.53275" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.29475" y1="17.81175" x2="199.67575" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="17.81175" x2="200.05675" y2="17.87525" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.13375" y1="17.87525" x2="182.27675" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="17.87525" x2="184.75325" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.87525" x2="185.07075" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.87525" x2="185.51525" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.73775" y1="17.87525" x2="187.92825" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="17.87525" x2="188.88075" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.24625" y1="17.87525" x2="192.56375" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="17.87525" x2="193.00825" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.87525" x2="193.77025" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="17.87525" x2="196.50075" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="17.87525" x2="197.32625" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="17.87525" x2="197.64375" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="17.87525" x2="198.34225" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.59625" y1="17.87525" x2="198.65975" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.48525" y1="17.87525" x2="199.73925" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="17.87525" x2="200.05675" y2="17.93875" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.19725" y1="17.93875" x2="182.27675" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="17.93875" x2="184.75325" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.94375" y1="17.93875" x2="185.07075" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.26125" y1="17.93875" x2="185.38825" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="17.93875" x2="185.51525" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="17.93875" x2="187.92825" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.69025" y1="17.93875" x2="188.81725" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="17.93875" x2="192.56375" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.94475" y1="17.93875" x2="193.00825" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="17.93875" x2="193.83375" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="17.93875" x2="196.50075" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="17.93875" x2="197.32625" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="17.93875" x2="197.64375" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="17.93875" x2="197.96125" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="17.93875" x2="198.08825" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="17.93875" x2="198.21525" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.34225" y1="17.93875" x2="198.40575" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.59625" y1="17.93875" x2="198.78675" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.97725" y1="17.93875" x2="199.16775" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="17.93875" x2="199.80275" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="17.93875" x2="200.05675" y2="18.00225" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.26075" y1="18.00225" x2="182.27675" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="18.00225" x2="184.75325" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.88025" y1="18.00225" x2="185.07075" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="18.00225" x2="185.45175" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.00225" x2="187.92825" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="18.00225" x2="188.81725" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="18.00225" x2="192.62725" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="18.00225" x2="193.07175" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.00225" x2="193.70675" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="18.00225" x2="193.83375" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.00225" x2="196.43725" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.00225" x2="197.07225" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="18.00225" x2="197.58025" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="18.00225" x2="198.08825" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.00225" x2="198.21525" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.46925" y1="18.00225" x2="198.72325" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.78675" y1="18.00225" x2="199.23125" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.42175" y1="18.00225" x2="199.48525" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="18.00225" x2="199.99325" y2="18.06575" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.26075" y1="18.06575" x2="182.21325" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="18.06575" x2="185.13425" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.19775" y1="18.06575" x2="185.45175" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="18.06575" x2="187.99175" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.62675" y1="18.06575" x2="188.81725" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.30975" y1="18.06575" x2="192.62725" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="18.06575" x2="193.07175" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.06575" x2="193.70675" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="18.06575" x2="193.83375" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.06575" x2="196.43725" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.06575" x2="197.00875" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="18.06575" x2="197.58025" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="18.06575" x2="197.89775" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="18.06575" x2="198.02475" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.06575" x2="198.21525" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.59625" y1="18.06575" x2="198.65975" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="18.06575" x2="199.35825" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.54875" y1="18.06575" x2="199.61225" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.80275" y1="18.06575" x2="200.05675" y2="18.12925" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.32425" y1="18.12925" x2="182.08625" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="18.12925" x2="185.19775" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.32475" y1="18.12925" x2="185.38825" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="18.12925" x2="188.24575" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.56325" y1="18.12925" x2="189.19825" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="18.12925" x2="192.69075" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="18.12925" x2="193.07175" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.12925" x2="193.70675" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="18.12925" x2="193.83375" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.12925" x2="196.43725" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.12925" x2="197.00875" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="18.12925" x2="197.58025" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="18.12925" x2="197.96125" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.12925" x2="198.21525" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="18.12925" x2="198.91375" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.16775" y1="18.12925" x2="199.48525" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.67575" y1="18.12925" x2="199.80275" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.92975" y1="18.12925" x2="200.18375" y2="18.19275" layer="94" grouprefs="LOGO"/>
<rectangle x1="181.45125" y1="18.19275" x2="181.95925" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.11825" y1="18.19275" x2="185.38825" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="18.19275" x2="188.37275" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.56325" y1="18.19275" x2="189.19825" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="18.19275" x2="192.69075" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="18.19275" x2="193.13525" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.19275" x2="193.70675" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="18.19275" x2="193.83375" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.19275" x2="196.43725" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.19275" x2="197.00875" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="18.19275" x2="197.51675" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="18.19275" x2="197.83425" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="18.19275" x2="197.96125" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.19275" x2="198.21525" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.85025" y1="18.19275" x2="198.91375" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.35825" y1="18.19275" x2="199.61225" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.80275" y1="18.19275" x2="199.92975" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.05675" y1="18.19275" x2="200.31075" y2="18.25625" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.18175" y1="18.25625" x2="185.32475" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.25625" x2="188.37275" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.56325" y1="18.25625" x2="189.26175" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.37325" y1="18.25625" x2="192.69075" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.07175" y1="18.25625" x2="193.13525" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.25625" x2="193.77025" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.25625" x2="196.43725" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.25625" x2="197.00875" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="18.25625" x2="197.51675" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="18.25625" x2="197.83425" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="18.25625" x2="197.96125" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.25625" x2="198.21525" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.91375" y1="18.25625" x2="198.97725" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.48525" y1="18.25625" x2="199.73925" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.86625" y1="18.25625" x2="200.05675" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.18375" y1="18.25625" x2="200.43775" y2="18.31975" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.24525" y1="18.31975" x2="185.26125" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.31975" x2="188.43625" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.31975" x2="189.26175" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="18.31975" x2="192.75425" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="18.31975" x2="193.19875" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.31975" x2="193.77025" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.31975" x2="196.43725" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.31975" x2="197.00875" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="18.31975" x2="197.45325" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="18.31975" x2="197.77075" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="18.31975" x2="197.89775" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.31975" x2="198.21525" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.04075" y1="18.31975" x2="199.16775" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="18.31975" x2="199.80275" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="18.31975" x2="200.18375" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.31075" y1="18.31975" x2="200.56475" y2="18.38325" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.37225" y1="18.38325" x2="185.19775" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.38325" x2="188.43625" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.38325" x2="189.19825" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.43675" y1="18.38325" x2="192.69075" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="18.38325" x2="193.19875" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.38325" x2="193.77025" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.38325" x2="196.43725" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.38325" x2="196.94525" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="18.38325" x2="197.45325" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="18.38325" x2="197.77075" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="18.38325" x2="197.89775" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.38325" x2="198.21525" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.23125" y1="18.38325" x2="199.29475" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="18.38325" x2="199.92975" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="18.38325" x2="200.31075" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.50125" y1="18.38325" x2="200.69175" y2="18.44675" layer="94" grouprefs="LOGO"/>
<rectangle x1="184.49925" y1="18.44675" x2="185.13425" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.44675" x2="189.00775" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="18.44675" x2="189.19825" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="18.44675" x2="192.75425" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.13525" y1="18.44675" x2="193.26225" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.44675" x2="193.70675" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.44675" x2="196.43725" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.44675" x2="196.94525" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="18.44675" x2="197.45325" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.44675" x2="197.70725" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="18.44675" x2="197.83425" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.44675" x2="198.21525" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.35825" y1="18.44675" x2="199.42175" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.86625" y1="18.44675" x2="200.05675" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="18.44675" x2="200.43775" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.62825" y1="18.44675" x2="200.81875" y2="18.51025" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.67425" y1="18.51025" x2="188.75375" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.81725" y1="18.51025" x2="189.19825" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="18.51025" x2="192.75425" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.19875" y1="18.51025" x2="193.26225" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.51025" x2="193.70675" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.51025" x2="196.43725" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.51025" x2="196.94525" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="18.51025" x2="197.38975" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.51025" x2="197.70725" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="18.51025" x2="197.83425" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.51025" x2="198.21525" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.48525" y1="18.51025" x2="199.54875" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="18.51025" x2="200.18375" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.37425" y1="18.51025" x2="200.56475" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.75525" y1="18.51025" x2="200.94575" y2="18.57375" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.61075" y1="18.57375" x2="188.69025" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="18.57375" x2="189.19825" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="18.57375" x2="192.75425" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.57375" x2="193.70675" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.57375" x2="196.43725" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.57375" x2="196.94525" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="18.57375" x2="197.38975" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="18.57375" x2="197.64375" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="18.57375" x2="197.77075" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.57375" x2="198.21525" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.61225" y1="18.57375" x2="199.67575" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="18.57375" x2="200.31075" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.50125" y1="18.57375" x2="200.69175" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.88225" y1="18.57375" x2="201.07275" y2="18.63725" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.54725" y1="18.63725" x2="188.37275" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.63725" x2="188.69025" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="18.63725" x2="189.07125" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="18.63725" x2="189.26175" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="18.63725" x2="192.81775" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.63725" x2="193.64325" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.63725" x2="196.43725" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.63725" x2="196.94525" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="18.63725" x2="197.32625" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="18.63725" x2="197.77075" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="18.63725" x2="198.27875" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.73925" y1="18.63725" x2="199.80275" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="18.63725" x2="200.37425" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.62825" y1="18.63725" x2="200.81875" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="18.63725" x2="201.19975" y2="18.70075" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.48375" y1="18.70075" x2="187.73775" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.70075" x2="188.30925" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.70075" x2="188.69025" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="18.70075" x2="189.19825" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.50025" y1="18.70075" x2="192.81775" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.70075" x2="193.64325" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.70075" x2="196.43725" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.70075" x2="196.94525" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="18.70075" x2="197.32625" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="18.70075" x2="197.51675" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.70075" x2="197.77075" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.21525" y1="18.70075" x2="198.27875" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.86625" y1="18.70075" x2="199.92975" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.37425" y1="18.70075" x2="200.50125" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.75525" y1="18.70075" x2="200.94575" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.13625" y1="18.70075" x2="201.32675" y2="18.76425" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.35675" y1="18.76425" x2="187.67425" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.76425" x2="188.37275" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.76425" x2="188.69025" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.13475" y1="18.76425" x2="189.19825" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.56375" y1="18.76425" x2="192.75425" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.76425" x2="193.64325" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.76425" x2="196.43725" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.76425" x2="196.94525" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="18.76425" x2="197.26275" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="18.76425" x2="197.45325" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="18.76425" x2="197.64375" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="18.76425" x2="197.77075" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="18.76425" x2="198.21525" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="199.99325" y1="18.76425" x2="200.05675" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.50125" y1="18.76425" x2="200.62825" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.88225" y1="18.76425" x2="201.07275" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="18.76425" x2="201.32675" y2="18.82775" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.29325" y1="18.82775" x2="187.61075" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.82775" x2="188.37275" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.49975" y1="18.82775" x2="188.69025" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.07125" y1="18.82775" x2="189.13475" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.56375" y1="18.82775" x2="192.75425" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="18.82775" x2="193.64325" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.82775" x2="196.43725" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.82775" x2="196.94525" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="18.82775" x2="197.26275" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="18.82775" x2="197.38975" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="18.82775" x2="197.58025" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.82775" x2="197.83425" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="18.82775" x2="198.15175" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.12025" y1="18.82775" x2="200.18375" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="18.82775" x2="200.75525" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.07275" y1="18.82775" x2="201.26325" y2="18.89125" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.22975" y1="18.89125" x2="187.48375" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.89125" x2="188.69025" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.88075" y1="18.89125" x2="189.13475" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.56375" y1="18.89125" x2="192.69075" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.45275" y1="18.89125" x2="193.64325" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.89125" x2="196.43725" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.89125" x2="196.94525" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="18.89125" x2="197.32625" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="18.89125" x2="197.51675" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.89125" x2="197.83425" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="18.89125" x2="198.15175" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.24725" y1="18.89125" x2="200.31075" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.69175" y1="18.89125" x2="200.88225" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="18.89125" x2="201.39025" y2="18.95475" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.16625" y1="18.95475" x2="187.42025" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="18.95475" x2="188.69025" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="18.95475" x2="189.13475" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.56375" y1="18.95475" x2="192.69075" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.45275" y1="18.95475" x2="193.64325" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="18.95475" x2="196.43725" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="18.95475" x2="196.94525" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="18.95475" x2="197.32625" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="18.95475" x2="197.45325" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="18.95475" x2="197.83425" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="18.95475" x2="198.08825" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.43775" y1="18.95475" x2="200.50125" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.81875" y1="18.95475" x2="200.94575" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.32675" y1="18.95475" x2="201.51725" y2="19.01825" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.10275" y1="19.01825" x2="187.35675" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.80125" y1="19.01825" x2="188.69025" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="19.01825" x2="189.13475" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.01825" x2="192.69075" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="19.01825" x2="193.64325" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="19.01825" x2="196.43725" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.01825" x2="196.94525" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="19.01825" x2="197.38975" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.01825" x2="197.83425" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="19.01825" x2="198.02475" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.56475" y1="19.01825" x2="200.62825" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.94575" y1="19.01825" x2="201.07275" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.45375" y1="19.01825" x2="201.70775" y2="19.08175" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.97575" y1="19.08175" x2="187.29325" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="19.08175" x2="188.62675" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="19.08175" x2="189.07125" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.08175" x2="192.69075" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.00825" y1="19.08175" x2="193.26225" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.45275" y1="19.08175" x2="193.64325" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="19.08175" x2="196.43725" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.08175" x2="196.94525" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="19.08175" x2="197.26275" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.08175" x2="197.77075" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="19.08175" x2="197.96125" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.69175" y1="19.08175" x2="200.75525" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.00925" y1="19.08175" x2="201.19975" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.58075" y1="19.08175" x2="201.83475" y2="19.14525" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.91225" y1="19.14525" x2="187.16625" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="19.14525" x2="188.62675" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="189.00775" y1="19.14525" x2="189.07125" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.69075" y1="19.14525" x2="192.75425" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.81775" y1="19.14525" x2="192.94475" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.51625" y1="19.14525" x2="193.57975" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="19.14525" x2="196.43725" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.14525" x2="196.94525" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="19.14525" x2="197.32625" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="19.14525" x2="197.58025" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.14525" x2="197.83425" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.81875" y1="19.14525" x2="200.88225" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.13625" y1="19.14525" x2="201.32675" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.70775" y1="19.14525" x2="201.96175" y2="19.20875" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.84875" y1="19.20875" x2="187.10275" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.86475" y1="19.20875" x2="188.69025" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.94425" y1="19.20875" x2="189.00775" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.69075" y1="19.20875" x2="192.75425" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="19.20875" x2="193.32575" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="19.20875" x2="196.50075" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.20875" x2="196.94525" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="19.20875" x2="197.32625" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.20875" x2="197.77075" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="200.94575" y1="19.20875" x2="201.00925" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="19.20875" x2="201.32675" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.83475" y1="19.20875" x2="202.08875" y2="19.27225" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.78525" y1="19.27225" x2="187.03925" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.92825" y1="19.27225" x2="188.94425" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.69075" y1="19.27225" x2="192.75425" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="19.27225" x2="193.13525" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="19.27225" x2="193.38925" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="19.27225" x2="196.50075" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.27225" x2="196.94525" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="19.27225" x2="197.58025" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.27225" x2="197.70725" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.07275" y1="19.27225" x2="201.13625" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="19.27225" x2="201.32675" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.96175" y1="19.27225" x2="202.15225" y2="19.33575" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.65825" y1="19.33575" x2="186.97575" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="187.99175" y1="19.33575" x2="188.94425" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="19.33575" x2="193.13525" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.32575" y1="19.33575" x2="193.38925" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="19.33575" x2="196.50075" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.33575" x2="196.94525" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="19.33575" x2="197.26275" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="19.33575" x2="197.58025" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.33575" x2="197.70725" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.19975" y1="19.33575" x2="201.32675" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.39025" y1="19.33575" x2="201.45375" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="202.02525" y1="19.33575" x2="202.21575" y2="19.39925" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.59475" y1="19.39925" x2="186.84875" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.05525" y1="19.39925" x2="188.88075" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.88125" y1="19.39925" x2="193.19875" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.38925" y1="19.39925" x2="193.45275" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="19.39925" x2="193.83375" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="19.39925" x2="196.50075" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.39925" x2="196.94525" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="19.39925" x2="197.58025" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.39925" x2="197.70725" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="19.39925" x2="198.02475" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.51725" y1="19.39925" x2="201.58075" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.96175" y1="19.39925" x2="202.15225" y2="19.46275" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.53125" y1="19.46275" x2="186.78525" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="188.24575" y1="19.46275" x2="188.75375" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.46275" x2="193.19875" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="19.46275" x2="193.77025" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="19.46275" x2="196.50075" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.88175" y1="19.46275" x2="196.94525" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="19.46275" x2="197.51675" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.46275" x2="197.64375" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="19.46275" x2="198.08825" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.64425" y1="19.46275" x2="201.70775" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.96175" y1="19.46275" x2="202.15225" y2="19.52625" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.46775" y1="19.52625" x2="186.59475" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.65825" y1="19.52625" x2="186.72175" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.52625" x2="193.19875" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="19.52625" x2="193.83375" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="19.52625" x2="196.50075" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.52625" x2="196.88175" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="19.52625" x2="197.51675" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.52625" x2="197.64375" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="19.52625" x2="198.15175" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.77125" y1="19.52625" x2="201.83475" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.96175" y1="19.52625" x2="202.08875" y2="19.58975" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.34075" y1="19.58975" x2="186.65825" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.58975" x2="193.19875" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="19.58975" x2="193.77025" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="19.58975" x2="196.56425" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.81825" y1="19.58975" x2="196.88175" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="19.58975" x2="197.45325" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="19.58975" x2="197.58025" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="19.58975" x2="197.96125" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="19.58975" x2="198.21525" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="201.89825" y1="19.58975" x2="202.02525" y2="19.65325" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.27725" y1="19.65325" x2="186.59475" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.65325" x2="193.26225" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="19.65325" x2="193.77025" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="19.65325" x2="196.62775" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="19.65325" x2="196.81825" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="19.65325" x2="197.45325" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="19.65325" x2="197.58025" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="19.65325" x2="197.89775" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.15175" y1="19.65325" x2="198.21525" y2="19.71675" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.21375" y1="19.71675" x2="186.46775" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.71675" x2="193.26225" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="19.71675" x2="193.83375" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="19.71675" x2="197.38975" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="19.71675" x2="197.51675" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="19.71675" x2="197.83425" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="19.71675" x2="198.15175" y2="19.78025" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.15025" y1="19.78025" x2="186.27725" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.34075" y1="19.78025" x2="186.40425" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.78025" x2="193.26225" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.83375" y1="19.78025" x2="193.96075" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="19.78025" x2="197.38975" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="19.78025" x2="197.51675" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="19.78025" x2="197.77075" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.08825" y1="19.78025" x2="198.15175" y2="19.84375" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.08675" y1="19.84375" x2="186.34075" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.84375" x2="193.13525" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.83375" y1="19.84375" x2="193.89725" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="19.84375" x2="197.32625" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="19.84375" x2="197.45325" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.84375" x2="197.77075" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="19.84375" x2="198.08825" y2="19.90725" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.95975" y1="19.90725" x2="186.21375" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.90725" x2="193.07175" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="19.90725" x2="193.32575" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="19.90725" x2="193.83375" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="19.90725" x2="197.32625" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="19.90725" x2="197.45325" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="19.90725" x2="197.70725" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="198.02475" y1="19.90725" x2="198.08825" y2="19.97075" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.89625" y1="19.97075" x2="186.15025" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="19.97075" x2="193.07175" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.26225" y1="19.97075" x2="193.38925" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="19.97075" x2="193.83375" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="19.97075" x2="196.37375" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="19.97075" x2="197.38975" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.51675" y1="19.97075" x2="197.64375" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="19.97075" x2="198.02475" y2="20.03425" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="20.03425" x2="185.95975" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="186.02325" y1="20.03425" x2="186.08675" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="20.03425" x2="193.45275" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="20.03425" x2="193.89725" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.03425" x2="196.37375" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="20.03425" x2="197.26275" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="20.03425" x2="197.38975" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.45325" y1="20.03425" x2="197.58025" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.96125" y1="20.03425" x2="198.02475" y2="20.09775" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.64225" y1="20.09775" x2="186.02325" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="20.09775" x2="193.64325" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.77025" y1="20.09775" x2="193.83375" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.09775" x2="196.43725" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="20.09775" x2="197.32625" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="20.09775" x2="197.51675" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="20.09775" x2="197.96125" y2="20.16125" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="20.16125" x2="185.76925" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.89625" y1="20.16125" x2="185.95975" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="20.16125" x2="193.64325" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="193.70675" y1="20.16125" x2="193.83375" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.16125" x2="196.43725" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="20.16125" x2="197.19925" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="20.16125" x2="197.51675" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.89775" y1="20.16125" x2="197.96125" y2="20.22475" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.57875" y1="20.22475" x2="185.70575" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.89625" y1="20.22475" x2="185.95975" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="20.22475" x2="193.77025" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.18325" y1="20.22475" x2="196.50075" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="20.22475" x2="197.45325" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="20.22475" x2="197.89775" y2="20.28825" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.51525" y1="20.28825" x2="185.64225" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.76925" y1="20.28825" x2="185.83275" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.62725" y1="20.28825" x2="193.77025" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.18325" y1="20.28825" x2="196.37375" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="20.28825" x2="196.50075" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.94525" y1="20.28825" x2="197.07225" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="20.28825" x2="197.38975" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.83425" y1="20.28825" x2="197.89775" y2="20.35175" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="20.35175" x2="185.70575" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.69075" y1="20.35175" x2="193.77025" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.18325" y1="20.35175" x2="196.37375" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.50075" y1="20.35175" x2="196.56425" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="20.35175" x2="197.07225" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.19925" y1="20.35175" x2="197.32625" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.77075" y1="20.35175" x2="197.83425" y2="20.41525" layer="94" grouprefs="LOGO"/>
<rectangle x1="185.45175" y1="20.41525" x2="185.57875" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="192.75425" y1="20.41525" x2="193.70675" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.41525" x2="196.37375" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.50075" y1="20.41525" x2="196.62775" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="20.41525" x2="197.32625" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.70725" y1="20.41525" x2="197.77075" y2="20.47875" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.47875" x2="196.31025" y2="20.54225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="20.47875" x2="196.62775" y2="20.54225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.75475" y1="20.47875" x2="196.94525" y2="20.54225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.07225" y1="20.47875" x2="197.26275" y2="20.54225" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.64375" y1="20.47875" x2="197.70725" y2="20.54225" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.24675" y1="20.54225" x2="196.37375" y2="20.60575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="20.54225" x2="196.62775" y2="20.60575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="20.54225" x2="196.81825" y2="20.60575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="20.54225" x2="197.19925" y2="20.60575" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="20.54225" x2="197.70725" y2="20.60575" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="20.60575" x2="196.37375" y2="20.66925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.69125" y1="20.60575" x2="196.75475" y2="20.66925" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.58025" y1="20.60575" x2="197.64375" y2="20.66925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="20.66925" x2="196.37375" y2="20.73275" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="20.66925" x2="197.58025" y2="20.73275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.31025" y1="20.73275" x2="196.43725" y2="20.79625" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.38975" y1="20.73275" x2="197.51675" y2="20.79625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="20.79625" x2="196.43725" y2="20.85975" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.32625" y1="20.79625" x2="197.38975" y2="20.85975" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.37375" y1="20.85975" x2="196.43725" y2="20.92325" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.26275" y1="20.85975" x2="197.32625" y2="20.92325" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.43725" y1="20.92325" x2="196.50075" y2="20.98675" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.13575" y1="20.92325" x2="197.26275" y2="20.98675" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.50075" y1="20.98675" x2="196.62775" y2="21.05025" layer="94" grouprefs="LOGO"/>
<rectangle x1="197.00875" y1="20.98675" x2="197.13575" y2="21.05025" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="21.05025" x2="196.62775" y2="21.11375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.88175" y1="21.05025" x2="196.94525" y2="21.11375" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.62775" y1="21.36775" x2="196.69125" y2="21.43125" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="21.87575" x2="196.62775" y2="21.93925" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="21.93925" x2="196.62775" y2="22.00275" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="22.00275" x2="196.69125" y2="22.06625" layer="94" grouprefs="LOGO"/>
<rectangle x1="196.56425" y1="22.06625" x2="196.62775" y2="22.12975" layer="94" grouprefs="LOGO"/>
<text x="225.1075" y="20.701" size="1.6764" layer="94" font="vector">Carte mat de balise</text>
</plain>
<instances>
<instance part="SUPPLY37" gate="GND" x="33.02" y="43.18" smashed="yes" rot="R90">
<attribute name="VALUE" x="36.195" y="41.275" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SDA" gate="G$1" x="68.58" y="45.72" smashed="yes">
<attribute name="NAME" x="72.136" y="41.148" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="74.295" y="41.148" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="RSDA" gate="G$1" x="68.58" y="35.56" smashed="yes" rot="R90">
<attribute name="NAME" x="67.0814" y="31.75" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="71.882" y="31.75" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SCL" gate="G$1" x="76.2" y="45.72" smashed="yes">
<attribute name="NAME" x="79.756" y="41.148" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="81.915" y="41.148" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="RSCL" gate="G$1" x="76.2" y="35.56" smashed="yes" rot="R90">
<attribute name="NAME" x="74.7014" y="31.75" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="79.502" y="31.75" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="VCC" gate="G$1" x="60.96" y="45.72" smashed="yes">
<attribute name="NAME" x="64.516" y="41.148" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="66.675" y="41.148" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="RVCC" gate="G$1" x="60.96" y="35.56" smashed="yes" rot="R90">
<attribute name="NAME" x="59.4614" y="31.75" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="64.262" y="31.75" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY38" gate="GND" x="68.58" y="27.94" smashed="yes">
<attribute name="VALUE" x="66.675" y="24.765" size="1.778" layer="96"/>
</instance>
<instance part="R4" gate="G$1" x="43.18" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="41.6814" y="19.05" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="46.482" y="19.05" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R5" gate="G$1" x="43.18" y="38.1" smashed="yes" rot="R90">
<attribute name="NAME" x="41.6814" y="34.29" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="46.482" y="34.29" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="FRAME1" gate="G$1" x="0" y="0" smashed="yes">
<attribute name="DRAWING_NAME" x="217.17" y="15.24" size="2.54" layer="94"/>
<attribute name="LAST_DATE_TIME" x="217.17" y="10.16" size="2.286" layer="94"/>
<attribute name="SHEET" x="230.505" y="5.08" size="2.54" layer="94"/>
</instance>
<instance part="S1" gate="G$1" x="246.38" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="251.46" y="154.94" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="254" y="154.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY5" gate="GND" x="236.22" y="165.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="233.045" y="167.005" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S2" gate="G$1" x="246.38" y="124.46" smashed="yes" rot="R90">
<attribute name="NAME" x="251.46" y="121.92" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="254" y="121.92" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY2" gate="GND" x="236.22" y="132.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="233.045" y="133.985" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S3" gate="G$1" x="246.38" y="91.44" smashed="yes" rot="R90">
<attribute name="NAME" x="251.46" y="88.9" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="254" y="88.9" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY8" gate="GND" x="236.22" y="99.06" smashed="yes" rot="R270">
<attribute name="VALUE" x="233.045" y="100.965" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S4" gate="G$1" x="246.38" y="58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="251.46" y="55.88" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="254" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY11" gate="GND" x="236.22" y="66.04" smashed="yes" rot="R270">
<attribute name="VALUE" x="233.045" y="67.945" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S5" gate="G$1" x="220.98" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="226.06" y="154.94" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="228.6" y="154.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY14" gate="GND" x="210.82" y="165.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="207.645" y="167.005" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S6" gate="G$1" x="220.98" y="124.46" smashed="yes" rot="R90">
<attribute name="NAME" x="226.06" y="121.92" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="228.6" y="121.92" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY17" gate="GND" x="210.82" y="132.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="207.645" y="133.985" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S7" gate="G$1" x="220.98" y="91.44" smashed="yes" rot="R90">
<attribute name="NAME" x="226.06" y="88.9" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="228.6" y="88.9" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY20" gate="GND" x="210.82" y="99.06" smashed="yes" rot="R270">
<attribute name="VALUE" x="207.645" y="100.965" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S8" gate="G$1" x="220.98" y="58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="226.06" y="55.88" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="228.6" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY23" gate="GND" x="210.82" y="66.04" smashed="yes" rot="R270">
<attribute name="VALUE" x="207.645" y="67.945" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S9" gate="G$1" x="195.58" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="200.66" y="154.94" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="203.2" y="154.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY26" gate="GND" x="185.42" y="165.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="182.245" y="167.005" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S10" gate="G$1" x="195.58" y="124.46" smashed="yes" rot="R90">
<attribute name="NAME" x="200.66" y="121.92" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="203.2" y="121.92" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY29" gate="GND" x="185.42" y="132.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="182.245" y="133.985" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S11" gate="G$1" x="195.58" y="91.44" smashed="yes" rot="R90">
<attribute name="NAME" x="200.66" y="88.9" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="203.2" y="88.9" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY32" gate="GND" x="185.42" y="99.06" smashed="yes" rot="R270">
<attribute name="VALUE" x="182.245" y="100.965" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S12" gate="G$1" x="195.58" y="58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="200.66" y="55.88" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="203.2" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY35" gate="GND" x="185.42" y="66.04" smashed="yes" rot="R270">
<attribute name="VALUE" x="182.245" y="67.945" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S13" gate="G$1" x="170.18" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="175.26" y="154.94" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="177.8" y="154.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY44" gate="GND" x="160.02" y="165.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="156.845" y="167.005" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S14" gate="G$1" x="170.18" y="124.46" smashed="yes" rot="R90">
<attribute name="NAME" x="175.26" y="121.92" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="177.8" y="121.92" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY47" gate="GND" x="160.02" y="132.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="156.845" y="133.985" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S15" gate="G$1" x="170.18" y="91.44" smashed="yes" rot="R90">
<attribute name="NAME" x="175.26" y="88.9" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="177.8" y="88.9" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY50" gate="GND" x="160.02" y="99.06" smashed="yes" rot="R270">
<attribute name="VALUE" x="156.845" y="100.965" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="S16" gate="G$1" x="170.18" y="58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="175.26" y="55.88" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="177.8" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY53" gate="GND" x="160.02" y="66.04" smashed="yes" rot="R270">
<attribute name="VALUE" x="156.845" y="67.945" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUPPLY1" gate="G$1" x="33.02" y="22.86" smashed="yes" rot="R270">
<attribute name="VALUE" x="36.195" y="24.765" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUPPLY3" gate="G$1" x="160.02" y="170.18" smashed="yes" rot="MR270">
<attribute name="VALUE" x="156.845" y="172.085" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY4" gate="G$1" x="160.02" y="137.16" smashed="yes" rot="MR270">
<attribute name="VALUE" x="156.845" y="139.065" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY6" gate="G$1" x="160.02" y="104.14" smashed="yes" rot="MR270">
<attribute name="VALUE" x="156.845" y="106.045" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY7" gate="G$1" x="160.02" y="71.12" smashed="yes" rot="MR270">
<attribute name="VALUE" x="156.845" y="73.025" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY9" gate="G$1" x="185.42" y="170.18" smashed="yes" rot="MR270">
<attribute name="VALUE" x="182.245" y="172.085" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY10" gate="G$1" x="185.42" y="137.16" smashed="yes" rot="MR270">
<attribute name="VALUE" x="182.245" y="139.065" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY12" gate="G$1" x="185.42" y="104.14" smashed="yes" rot="MR270">
<attribute name="VALUE" x="182.245" y="106.045" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY13" gate="G$1" x="185.42" y="71.12" smashed="yes" rot="MR270">
<attribute name="VALUE" x="182.245" y="73.025" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY15" gate="G$1" x="210.82" y="170.18" smashed="yes" rot="MR270">
<attribute name="VALUE" x="207.645" y="172.085" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY16" gate="G$1" x="236.22" y="170.18" smashed="yes" rot="MR270">
<attribute name="VALUE" x="233.045" y="172.085" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY18" gate="G$1" x="236.22" y="137.16" smashed="yes" rot="MR270">
<attribute name="VALUE" x="233.045" y="139.065" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY19" gate="G$1" x="210.82" y="137.16" smashed="yes" rot="MR270">
<attribute name="VALUE" x="207.645" y="139.065" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY21" gate="G$1" x="210.82" y="104.14" smashed="yes" rot="MR270">
<attribute name="VALUE" x="207.645" y="106.045" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY22" gate="G$1" x="236.22" y="104.14" smashed="yes" rot="MR270">
<attribute name="VALUE" x="233.045" y="106.045" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY24" gate="G$1" x="236.22" y="71.12" smashed="yes" rot="MR270">
<attribute name="VALUE" x="233.045" y="73.025" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY25" gate="G$1" x="210.82" y="71.12" smashed="yes" rot="MR270">
<attribute name="VALUE" x="207.645" y="73.025" size="1.778" layer="96" rot="MR270"/>
</instance>
<instance part="SUPPLY28" gate="G$1" x="60.96" y="50.8" smashed="yes">
<attribute name="VALUE" x="59.055" y="53.975" size="1.778" layer="96"/>
</instance>
<instance part="J1" gate="G$1" x="22.86" y="33.02" smashed="yes" rot="R270">
<attribute name="NAME" x="17.78" y="38.1" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="15.24" y="38.1" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUPPLY31" gate="G$1" x="43.18" y="15.24" smashed="yes" rot="R180">
<attribute name="VALUE" x="45.085" y="12.065" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY33" gate="G$1" x="43.18" y="45.72" smashed="yes">
<attribute name="VALUE" x="41.275" y="48.895" size="1.778" layer="96"/>
</instance>
<instance part="SW1" gate="A" x="20.32" y="149.86" smashed="yes" rot="R180">
<attribute name="NAME" x="25.4" y="144.145" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="25.4" y="157.48" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R1" gate="G$1" x="30.48" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="28.9814" y="153.67" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="33.782" y="153.67" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R2" gate="G$1" x="38.1" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="36.6014" y="151.13" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="41.402" y="151.13" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R3" gate="G$1" x="45.72" y="152.4" smashed="yes" rot="R90">
<attribute name="NAME" x="44.2214" y="148.59" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="49.022" y="148.59" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY34" gate="GND" x="30.48" y="165.1" smashed="yes" rot="R180">
<attribute name="VALUE" x="32.385" y="168.275" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY36" gate="GND" x="38.1" y="162.56" smashed="yes" rot="R180">
<attribute name="VALUE" x="40.005" y="165.735" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY39" gate="GND" x="45.72" y="160.02" smashed="yes" rot="R180">
<attribute name="VALUE" x="47.625" y="163.195" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY40" gate="G$1" x="10.16" y="152.4" smashed="yes" rot="R90">
<attribute name="VALUE" x="6.985" y="150.495" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U1" gate="G$1" x="38.1" y="91.44" smashed="yes">
<attribute name="NAME" x="27.9206" y="120.4465" size="1.7814" layer="95"/>
<attribute name="VALUE" x="28.951509375" y="60.6921" size="1.77886875" layer="96"/>
</instance>
<instance part="U2" gate="G$1" x="104.14" y="91.44" smashed="yes">
<attribute name="NAME" x="93.9606" y="120.4465" size="1.7814" layer="95"/>
<attribute name="VALUE" x="94.991509375" y="60.6921" size="1.77886875" layer="96"/>
</instance>
<instance part="SW2" gate="A" x="88.9" y="149.86" smashed="yes" rot="R180">
<attribute name="NAME" x="93.98" y="144.145" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="93.98" y="157.48" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R6" gate="G$1" x="99.06" y="157.48" smashed="yes" rot="R90">
<attribute name="NAME" x="97.5614" y="153.67" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="102.362" y="153.67" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R7" gate="G$1" x="106.68" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="105.1814" y="151.13" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="109.982" y="151.13" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R8" gate="G$1" x="114.3" y="152.4" smashed="yes" rot="R90">
<attribute name="NAME" x="112.8014" y="148.59" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="117.602" y="148.59" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY27" gate="GND" x="99.06" y="165.1" smashed="yes" rot="R180">
<attribute name="VALUE" x="100.965" y="168.275" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY30" gate="GND" x="106.68" y="162.56" smashed="yes" rot="R180">
<attribute name="VALUE" x="108.585" y="165.735" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY41" gate="GND" x="114.3" y="160.02" smashed="yes" rot="R180">
<attribute name="VALUE" x="116.205" y="163.195" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUPPLY42" gate="G$1" x="78.74" y="152.4" smashed="yes" rot="R90">
<attribute name="VALUE" x="75.565" y="150.495" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="RESET2" gate="P" x="83.82" y="111.76" smashed="yes">
<attribute name="NAME" x="82.677" y="113.6142" size="1.778" layer="95"/>
<attribute name="VALUE" x="82.677" y="108.458" size="1.778" layer="96"/>
</instance>
<instance part="RESET1" gate="P" x="17.78" y="111.76" smashed="yes">
<attribute name="NAME" x="16.637" y="113.6142" size="1.778" layer="95"/>
<attribute name="VALUE" x="16.637" y="108.458" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY43" gate="GND" x="55.88" y="66.04" smashed="yes" rot="R90">
<attribute name="VALUE" x="59.055" y="64.135" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY45" gate="GND" x="121.92" y="66.04" smashed="yes" rot="R90">
<attribute name="VALUE" x="125.095" y="64.135" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY46" gate="G$1" x="55.88" y="116.84" smashed="yes" rot="R270">
<attribute name="VALUE" x="59.055" y="118.745" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUPPLY48" gate="G$1" x="121.92" y="116.84" smashed="yes" rot="R270">
<attribute name="VALUE" x="125.095" y="118.745" size="1.778" layer="96" rot="R270"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="RSDA" gate="G$1" pin="1"/>
<pinref part="SUPPLY38" gate="GND" pin="GND"/>
<pinref part="RVCC" gate="G$1" pin="1"/>
<wire x1="60.96" y1="30.48" x2="68.58" y2="30.48" width="0.1524" layer="91"/>
<junction x="68.58" y="30.48"/>
<pinref part="RSCL" gate="G$1" pin="1"/>
<wire x1="68.58" y1="30.48" x2="76.2" y2="30.48" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="2"/>
<pinref part="SUPPLY5" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S2" gate="G$1" pin="2"/>
<pinref part="SUPPLY2" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S3" gate="G$1" pin="2"/>
<pinref part="SUPPLY8" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S4" gate="G$1" pin="2"/>
<pinref part="SUPPLY11" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S5" gate="G$1" pin="2"/>
<pinref part="SUPPLY14" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S6" gate="G$1" pin="2"/>
<pinref part="SUPPLY17" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S7" gate="G$1" pin="2"/>
<pinref part="SUPPLY20" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S8" gate="G$1" pin="2"/>
<pinref part="SUPPLY23" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S9" gate="G$1" pin="2"/>
<pinref part="SUPPLY26" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S10" gate="G$1" pin="2"/>
<pinref part="SUPPLY29" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S11" gate="G$1" pin="2"/>
<pinref part="SUPPLY32" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S12" gate="G$1" pin="2"/>
<pinref part="SUPPLY35" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S13" gate="G$1" pin="2"/>
<pinref part="SUPPLY44" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S14" gate="G$1" pin="2"/>
<pinref part="SUPPLY47" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S15" gate="G$1" pin="2"/>
<pinref part="SUPPLY50" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="S16" gate="G$1" pin="2"/>
<pinref part="SUPPLY53" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="5"/>
<pinref part="SUPPLY37" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<pinref part="SUPPLY34" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="SUPPLY36" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<pinref part="SUPPLY39" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R6" gate="G$1" pin="2"/>
<pinref part="SUPPLY27" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R7" gate="G$1" pin="2"/>
<pinref part="SUPPLY30" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="R8" gate="G$1" pin="2"/>
<pinref part="SUPPLY41" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="SUPPLY43" gate="GND" pin="GND"/>
<pinref part="U1" gate="G$1" pin="VSS"/>
</segment>
<segment>
<pinref part="SUPPLY45" gate="GND" pin="GND"/>
<pinref part="U2" gate="G$1" pin="VSS"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="SCL" gate="G$1" pin="A"/>
<wire x1="76.2" y1="48.26" x2="76.2" y2="50.8" width="0.1524" layer="91"/>
<label x="76.2" y="50.8" size="1.778" layer="95" rot="R90"/>
</segment>
<segment>
<wire x1="30.48" y1="33.02" x2="43.18" y2="33.02" width="0.1524" layer="91"/>
<label x="48.26" y="33.02" size="1.778" layer="95"/>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="43.18" y1="33.02" x2="48.26" y2="33.02" width="0.1524" layer="91"/>
<junction x="43.18" y="33.02"/>
<pinref part="J1" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="4"/>
<wire x1="238.76" y1="154.94" x2="236.22" y2="154.94" width="0.1524" layer="91"/>
<label x="236.22" y="154.94" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S2" gate="G$1" pin="4"/>
<wire x1="238.76" y1="121.92" x2="236.22" y2="121.92" width="0.1524" layer="91"/>
<label x="236.22" y="121.92" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S3" gate="G$1" pin="4"/>
<wire x1="238.76" y1="88.9" x2="236.22" y2="88.9" width="0.1524" layer="91"/>
<label x="236.22" y="88.9" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S4" gate="G$1" pin="4"/>
<wire x1="238.76" y1="55.88" x2="236.22" y2="55.88" width="0.1524" layer="91"/>
<label x="236.22" y="55.88" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S5" gate="G$1" pin="4"/>
<wire x1="213.36" y1="154.94" x2="210.82" y2="154.94" width="0.1524" layer="91"/>
<label x="210.82" y="154.94" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S6" gate="G$1" pin="4"/>
<wire x1="213.36" y1="121.92" x2="210.82" y2="121.92" width="0.1524" layer="91"/>
<label x="210.82" y="121.92" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S7" gate="G$1" pin="4"/>
<wire x1="213.36" y1="88.9" x2="210.82" y2="88.9" width="0.1524" layer="91"/>
<label x="210.82" y="88.9" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S8" gate="G$1" pin="4"/>
<wire x1="213.36" y1="55.88" x2="210.82" y2="55.88" width="0.1524" layer="91"/>
<label x="210.82" y="55.88" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S9" gate="G$1" pin="4"/>
<wire x1="187.96" y1="154.94" x2="185.42" y2="154.94" width="0.1524" layer="91"/>
<label x="185.42" y="154.94" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S10" gate="G$1" pin="4"/>
<wire x1="187.96" y1="121.92" x2="185.42" y2="121.92" width="0.1524" layer="91"/>
<label x="185.42" y="121.92" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S11" gate="G$1" pin="4"/>
<wire x1="187.96" y1="88.9" x2="185.42" y2="88.9" width="0.1524" layer="91"/>
<label x="185.42" y="88.9" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S12" gate="G$1" pin="4"/>
<wire x1="187.96" y1="55.88" x2="185.42" y2="55.88" width="0.1524" layer="91"/>
<label x="185.42" y="55.88" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S13" gate="G$1" pin="4"/>
<wire x1="162.56" y1="154.94" x2="160.02" y2="154.94" width="0.1524" layer="91"/>
<label x="160.02" y="154.94" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S14" gate="G$1" pin="4"/>
<wire x1="162.56" y1="121.92" x2="160.02" y2="121.92" width="0.1524" layer="91"/>
<label x="160.02" y="121.92" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S15" gate="G$1" pin="4"/>
<wire x1="162.56" y1="88.9" x2="160.02" y2="88.9" width="0.1524" layer="91"/>
<label x="160.02" y="88.9" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S16" gate="G$1" pin="4"/>
<wire x1="162.56" y1="55.88" x2="160.02" y2="55.88" width="0.1524" layer="91"/>
<label x="160.02" y="55.88" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="SCL"/>
<wire x1="22.86" y1="106.68" x2="20.32" y2="106.68" width="0.1524" layer="91"/>
<label x="15.24" y="106.68" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="SCL"/>
<wire x1="88.9" y1="106.68" x2="86.36" y2="106.68" width="0.1524" layer="91"/>
<label x="81.28" y="106.68" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="SDA" gate="G$1" pin="C"/>
<pinref part="RSDA" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="SCL" gate="G$1" pin="C"/>
<pinref part="RSCL" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="VCC" gate="G$1" pin="C"/>
<pinref part="RVCC" gate="G$1" pin="2"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="SDA" gate="G$1" pin="A"/>
<wire x1="68.58" y1="48.26" x2="68.58" y2="50.8" width="0.1524" layer="91"/>
<label x="68.58" y="50.8" size="1.778" layer="95" rot="R90"/>
</segment>
<segment>
<wire x1="30.48" y1="27.94" x2="43.18" y2="27.94" width="0.1524" layer="91"/>
<label x="48.26" y="27.94" size="1.778" layer="95"/>
<pinref part="R4" gate="G$1" pin="2"/>
<wire x1="43.18" y1="27.94" x2="48.26" y2="27.94" width="0.1524" layer="91"/>
<junction x="43.18" y="27.94"/>
<pinref part="J1" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="3"/>
<wire x1="238.76" y1="160.02" x2="236.22" y2="160.02" width="0.1524" layer="91"/>
<label x="236.22" y="160.02" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S2" gate="G$1" pin="3"/>
<wire x1="238.76" y1="127" x2="236.22" y2="127" width="0.1524" layer="91"/>
<label x="236.22" y="127" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S3" gate="G$1" pin="3"/>
<wire x1="238.76" y1="93.98" x2="236.22" y2="93.98" width="0.1524" layer="91"/>
<label x="236.22" y="93.98" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S4" gate="G$1" pin="3"/>
<wire x1="238.76" y1="60.96" x2="236.22" y2="60.96" width="0.1524" layer="91"/>
<label x="236.22" y="60.96" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S5" gate="G$1" pin="3"/>
<wire x1="213.36" y1="160.02" x2="210.82" y2="160.02" width="0.1524" layer="91"/>
<label x="210.82" y="160.02" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S6" gate="G$1" pin="3"/>
<wire x1="213.36" y1="127" x2="210.82" y2="127" width="0.1524" layer="91"/>
<label x="210.82" y="127" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S7" gate="G$1" pin="3"/>
<wire x1="213.36" y1="93.98" x2="210.82" y2="93.98" width="0.1524" layer="91"/>
<label x="210.82" y="93.98" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S8" gate="G$1" pin="3"/>
<wire x1="213.36" y1="60.96" x2="210.82" y2="60.96" width="0.1524" layer="91"/>
<label x="210.82" y="60.96" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S9" gate="G$1" pin="3"/>
<wire x1="187.96" y1="160.02" x2="185.42" y2="160.02" width="0.1524" layer="91"/>
<label x="185.42" y="160.02" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S10" gate="G$1" pin="3"/>
<wire x1="187.96" y1="127" x2="185.42" y2="127" width="0.1524" layer="91"/>
<label x="185.42" y="127" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S11" gate="G$1" pin="3"/>
<wire x1="187.96" y1="93.98" x2="185.42" y2="93.98" width="0.1524" layer="91"/>
<label x="185.42" y="93.98" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S12" gate="G$1" pin="3"/>
<wire x1="187.96" y1="60.96" x2="185.42" y2="60.96" width="0.1524" layer="91"/>
<label x="185.42" y="60.96" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S13" gate="G$1" pin="3"/>
<wire x1="162.56" y1="160.02" x2="160.02" y2="160.02" width="0.1524" layer="91"/>
<label x="160.02" y="160.02" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S14" gate="G$1" pin="3"/>
<wire x1="162.56" y1="127" x2="160.02" y2="127" width="0.1524" layer="91"/>
<label x="160.02" y="127" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S15" gate="G$1" pin="3"/>
<wire x1="162.56" y1="93.98" x2="160.02" y2="93.98" width="0.1524" layer="91"/>
<label x="160.02" y="93.98" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="S16" gate="G$1" pin="3"/>
<wire x1="162.56" y1="60.96" x2="160.02" y2="60.96" width="0.1524" layer="91"/>
<label x="160.02" y="60.96" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="SDA"/>
<wire x1="22.86" y1="104.14" x2="20.32" y2="104.14" width="0.1524" layer="91"/>
<label x="15.24" y="104.14" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="SDA"/>
<wire x1="88.9" y1="104.14" x2="86.36" y2="104.14" width="0.1524" layer="91"/>
<label x="81.28" y="104.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO1" class="0">
<segment>
<pinref part="S1" gate="G$1" pin="6"/>
<wire x1="238.76" y1="144.78" x2="236.22" y2="144.78" width="0.1524" layer="91"/>
<label x="236.22" y="144.78" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA0"/>
<wire x1="88.9" y1="88.9" x2="86.36" y2="88.9" width="0.1524" layer="91"/>
<label x="78.74" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO2" class="0">
<segment>
<pinref part="S2" gate="G$1" pin="6"/>
<wire x1="238.76" y1="111.76" x2="236.22" y2="111.76" width="0.1524" layer="91"/>
<label x="236.22" y="111.76" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA1"/>
<wire x1="88.9" y1="86.36" x2="86.36" y2="86.36" width="0.1524" layer="91"/>
<label x="78.74" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO3" class="0">
<segment>
<pinref part="S3" gate="G$1" pin="6"/>
<wire x1="238.76" y1="78.74" x2="236.22" y2="78.74" width="0.1524" layer="91"/>
<label x="236.22" y="78.74" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA2"/>
<wire x1="88.9" y1="83.82" x2="86.36" y2="83.82" width="0.1524" layer="91"/>
<label x="78.74" y="83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO4" class="0">
<segment>
<pinref part="S4" gate="G$1" pin="6"/>
<wire x1="238.76" y1="45.72" x2="236.22" y2="45.72" width="0.1524" layer="91"/>
<label x="236.22" y="45.72" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA3"/>
<wire x1="88.9" y1="81.28" x2="86.36" y2="81.28" width="0.1524" layer="91"/>
<label x="78.74" y="81.28" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO5" class="0">
<segment>
<pinref part="S5" gate="G$1" pin="6"/>
<wire x1="213.36" y1="144.78" x2="210.82" y2="144.78" width="0.1524" layer="91"/>
<label x="210.82" y="144.78" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA4"/>
<wire x1="88.9" y1="78.74" x2="86.36" y2="78.74" width="0.1524" layer="91"/>
<label x="78.74" y="78.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO6" class="0">
<segment>
<pinref part="S6" gate="G$1" pin="6"/>
<wire x1="213.36" y1="111.76" x2="210.82" y2="111.76" width="0.1524" layer="91"/>
<label x="210.82" y="111.76" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA5"/>
<wire x1="88.9" y1="76.2" x2="86.36" y2="76.2" width="0.1524" layer="91"/>
<label x="78.74" y="76.2" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO7" class="0">
<segment>
<pinref part="S7" gate="G$1" pin="6"/>
<wire x1="213.36" y1="78.74" x2="210.82" y2="78.74" width="0.1524" layer="91"/>
<label x="210.82" y="78.74" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA6"/>
<wire x1="88.9" y1="73.66" x2="86.36" y2="73.66" width="0.1524" layer="91"/>
<label x="78.74" y="73.66" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO8" class="0">
<segment>
<pinref part="S8" gate="G$1" pin="6"/>
<wire x1="213.36" y1="45.72" x2="210.82" y2="45.72" width="0.1524" layer="91"/>
<label x="210.82" y="45.72" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPA7"/>
<wire x1="88.9" y1="71.12" x2="86.36" y2="71.12" width="0.1524" layer="91"/>
<label x="78.74" y="71.12" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO9" class="0">
<segment>
<pinref part="S9" gate="G$1" pin="6"/>
<wire x1="187.96" y1="144.78" x2="185.42" y2="144.78" width="0.1524" layer="91"/>
<label x="185.42" y="144.78" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB0"/>
<wire x1="119.38" y1="88.9" x2="121.92" y2="88.9" width="0.1524" layer="91"/>
<label x="121.92" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO10" class="0">
<segment>
<pinref part="S10" gate="G$1" pin="6"/>
<wire x1="187.96" y1="111.76" x2="185.42" y2="111.76" width="0.1524" layer="91"/>
<label x="185.42" y="111.76" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB1"/>
<wire x1="119.38" y1="86.36" x2="121.92" y2="86.36" width="0.1524" layer="91"/>
<label x="121.92" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO11" class="0">
<segment>
<pinref part="S11" gate="G$1" pin="6"/>
<wire x1="187.96" y1="78.74" x2="185.42" y2="78.74" width="0.1524" layer="91"/>
<label x="185.42" y="78.74" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB2"/>
<wire x1="119.38" y1="83.82" x2="121.92" y2="83.82" width="0.1524" layer="91"/>
<label x="121.92" y="83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO12" class="0">
<segment>
<pinref part="S12" gate="G$1" pin="6"/>
<wire x1="187.96" y1="45.72" x2="185.42" y2="45.72" width="0.1524" layer="91"/>
<label x="185.42" y="45.72" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB3"/>
<wire x1="119.38" y1="81.28" x2="121.92" y2="81.28" width="0.1524" layer="91"/>
<label x="121.92" y="81.28" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO13" class="0">
<segment>
<pinref part="S13" gate="G$1" pin="6"/>
<wire x1="162.56" y1="144.78" x2="160.02" y2="144.78" width="0.1524" layer="91"/>
<label x="160.02" y="144.78" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB4"/>
<wire x1="119.38" y1="78.74" x2="121.92" y2="78.74" width="0.1524" layer="91"/>
<label x="121.92" y="78.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO14" class="0">
<segment>
<pinref part="S14" gate="G$1" pin="6"/>
<wire x1="162.56" y1="111.76" x2="160.02" y2="111.76" width="0.1524" layer="91"/>
<label x="160.02" y="111.76" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB5"/>
<wire x1="119.38" y1="76.2" x2="121.92" y2="76.2" width="0.1524" layer="91"/>
<label x="121.92" y="76.2" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO15" class="0">
<segment>
<pinref part="S15" gate="G$1" pin="6"/>
<wire x1="162.56" y1="78.74" x2="160.02" y2="78.74" width="0.1524" layer="91"/>
<label x="160.02" y="78.74" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB6"/>
<wire x1="119.38" y1="73.66" x2="121.92" y2="73.66" width="0.1524" layer="91"/>
<label x="121.92" y="73.66" size="1.778" layer="95"/>
</segment>
</net>
<net name="GPIO16" class="0">
<segment>
<pinref part="S16" gate="G$1" pin="6"/>
<wire x1="162.56" y1="45.72" x2="160.02" y2="45.72" width="0.1524" layer="91"/>
<label x="160.02" y="45.72" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="GPB7"/>
<wire x1="119.38" y1="71.12" x2="121.92" y2="71.12" width="0.1524" layer="91"/>
<label x="121.92" y="71.12" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT1" class="0">
<segment>
<pinref part="S1" gate="G$1" pin="5"/>
<wire x1="238.76" y1="149.86" x2="236.22" y2="149.86" width="0.1524" layer="91"/>
<label x="236.22" y="149.86" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA0"/>
<wire x1="22.86" y1="88.9" x2="20.32" y2="88.9" width="0.1524" layer="91"/>
<label x="10.16" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT2" class="0">
<segment>
<pinref part="S2" gate="G$1" pin="5"/>
<wire x1="238.76" y1="116.84" x2="236.22" y2="116.84" width="0.1524" layer="91"/>
<label x="236.22" y="116.84" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA1"/>
<wire x1="22.86" y1="86.36" x2="20.32" y2="86.36" width="0.1524" layer="91"/>
<label x="10.16" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT5" class="0">
<segment>
<pinref part="S5" gate="G$1" pin="5"/>
<wire x1="213.36" y1="149.86" x2="210.82" y2="149.86" width="0.1524" layer="91"/>
<label x="210.82" y="149.86" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA4"/>
<wire x1="22.86" y1="78.74" x2="20.32" y2="78.74" width="0.1524" layer="91"/>
<label x="10.16" y="78.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT9" class="0">
<segment>
<pinref part="S9" gate="G$1" pin="5"/>
<wire x1="187.96" y1="149.86" x2="185.42" y2="149.86" width="0.1524" layer="91"/>
<label x="185.42" y="149.86" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB0"/>
<wire x1="53.34" y1="88.9" x2="55.88" y2="88.9" width="0.1524" layer="91"/>
<label x="55.88" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT13" class="0">
<segment>
<pinref part="S13" gate="G$1" pin="5"/>
<wire x1="162.56" y1="149.86" x2="160.02" y2="149.86" width="0.1524" layer="91"/>
<label x="160.02" y="149.86" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB4"/>
<wire x1="53.34" y1="78.74" x2="55.88" y2="78.74" width="0.1524" layer="91"/>
<label x="55.88" y="78.74" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT14" class="0">
<segment>
<pinref part="S14" gate="G$1" pin="5"/>
<wire x1="162.56" y1="116.84" x2="160.02" y2="116.84" width="0.1524" layer="91"/>
<label x="160.02" y="116.84" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB5"/>
<wire x1="53.34" y1="76.2" x2="55.88" y2="76.2" width="0.1524" layer="91"/>
<label x="55.88" y="76.2" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT10" class="0">
<segment>
<pinref part="S10" gate="G$1" pin="5"/>
<wire x1="187.96" y1="116.84" x2="185.42" y2="116.84" width="0.1524" layer="91"/>
<label x="185.42" y="116.84" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB1"/>
<wire x1="53.34" y1="86.36" x2="55.88" y2="86.36" width="0.1524" layer="91"/>
<label x="55.88" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT6" class="0">
<segment>
<pinref part="S6" gate="G$1" pin="5"/>
<wire x1="213.36" y1="116.84" x2="210.82" y2="116.84" width="0.1524" layer="91"/>
<label x="210.82" y="116.84" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA5"/>
<wire x1="22.86" y1="76.2" x2="20.32" y2="76.2" width="0.1524" layer="91"/>
<label x="10.16" y="76.2" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT3" class="0">
<segment>
<pinref part="S3" gate="G$1" pin="5"/>
<wire x1="238.76" y1="83.82" x2="236.22" y2="83.82" width="0.1524" layer="91"/>
<label x="236.22" y="83.82" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA2"/>
<wire x1="22.86" y1="83.82" x2="20.32" y2="83.82" width="0.1524" layer="91"/>
<label x="10.16" y="83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT7" class="0">
<segment>
<pinref part="S7" gate="G$1" pin="5"/>
<wire x1="213.36" y1="83.82" x2="210.82" y2="83.82" width="0.1524" layer="91"/>
<label x="210.82" y="83.82" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA6"/>
<wire x1="22.86" y1="73.66" x2="20.32" y2="73.66" width="0.1524" layer="91"/>
<label x="10.16" y="73.66" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT11" class="0">
<segment>
<pinref part="S11" gate="G$1" pin="5"/>
<wire x1="187.96" y1="83.82" x2="185.42" y2="83.82" width="0.1524" layer="91"/>
<label x="185.42" y="83.82" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB2"/>
<wire x1="53.34" y1="83.82" x2="55.88" y2="83.82" width="0.1524" layer="91"/>
<label x="55.88" y="83.82" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT15" class="0">
<segment>
<pinref part="S15" gate="G$1" pin="5"/>
<wire x1="162.56" y1="83.82" x2="160.02" y2="83.82" width="0.1524" layer="91"/>
<label x="160.02" y="83.82" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB6"/>
<wire x1="53.34" y1="73.66" x2="55.88" y2="73.66" width="0.1524" layer="91"/>
<label x="55.88" y="73.66" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT16" class="0">
<segment>
<pinref part="S16" gate="G$1" pin="5"/>
<wire x1="162.56" y1="50.8" x2="160.02" y2="50.8" width="0.1524" layer="91"/>
<label x="160.02" y="50.8" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB7"/>
<wire x1="53.34" y1="71.12" x2="55.88" y2="71.12" width="0.1524" layer="91"/>
<label x="55.88" y="71.12" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT12" class="0">
<segment>
<pinref part="S12" gate="G$1" pin="5"/>
<wire x1="187.96" y1="50.8" x2="185.42" y2="50.8" width="0.1524" layer="91"/>
<label x="185.42" y="50.8" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPB3"/>
<wire x1="53.34" y1="81.28" x2="55.88" y2="81.28" width="0.1524" layer="91"/>
<label x="55.88" y="81.28" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT8" class="0">
<segment>
<pinref part="S8" gate="G$1" pin="5"/>
<wire x1="213.36" y1="50.8" x2="210.82" y2="50.8" width="0.1524" layer="91"/>
<label x="210.82" y="50.8" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA7"/>
<wire x1="22.86" y1="71.12" x2="20.32" y2="71.12" width="0.1524" layer="91"/>
<label x="10.16" y="71.12" size="1.778" layer="95"/>
</segment>
</net>
<net name="XSHUT4" class="0">
<segment>
<pinref part="S4" gate="G$1" pin="5"/>
<wire x1="238.76" y1="50.8" x2="236.22" y2="50.8" width="0.1524" layer="91"/>
<label x="236.22" y="50.8" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GPA3"/>
<wire x1="22.86" y1="81.28" x2="20.32" y2="81.28" width="0.1524" layer="91"/>
<label x="10.16" y="81.28" size="1.778" layer="95"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="S13" gate="G$1" pin="1"/>
<pinref part="SUPPLY3" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S14" gate="G$1" pin="1"/>
<pinref part="SUPPLY4" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S15" gate="G$1" pin="1"/>
<pinref part="SUPPLY6" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S16" gate="G$1" pin="1"/>
<pinref part="SUPPLY7" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S9" gate="G$1" pin="1"/>
<pinref part="SUPPLY9" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S10" gate="G$1" pin="1"/>
<pinref part="SUPPLY10" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S11" gate="G$1" pin="1"/>
<pinref part="SUPPLY12" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S12" gate="G$1" pin="1"/>
<pinref part="SUPPLY13" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S5" gate="G$1" pin="1"/>
<pinref part="SUPPLY15" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="1"/>
<pinref part="SUPPLY16" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S2" gate="G$1" pin="1"/>
<pinref part="SUPPLY18" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S6" gate="G$1" pin="1"/>
<pinref part="SUPPLY19" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S7" gate="G$1" pin="1"/>
<pinref part="SUPPLY21" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S3" gate="G$1" pin="1"/>
<pinref part="SUPPLY22" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S4" gate="G$1" pin="1"/>
<pinref part="SUPPLY24" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="S8" gate="G$1" pin="1"/>
<pinref part="SUPPLY25" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="VCC" gate="G$1" pin="A"/>
<pinref part="SUPPLY28" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="1"/>
<pinref part="SUPPLY1" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R4" gate="G$1" pin="1"/>
<pinref part="SUPPLY31" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R5" gate="G$1" pin="2"/>
<pinref part="SUPPLY33" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="SW1" gate="A" pin="1"/>
<pinref part="SUPPLY40" gate="G$1" pin="VCC"/>
<pinref part="SW1" gate="A" pin="3"/>
<pinref part="SW1" gate="A" pin="2"/>
<wire x1="12.7" y1="147.32" x2="12.7" y2="149.86" width="0.1524" layer="91"/>
<wire x1="12.7" y1="149.86" x2="12.7" y2="152.4" width="0.1524" layer="91"/>
<junction x="12.7" y="149.86"/>
<junction x="12.7" y="152.4"/>
</segment>
<segment>
<pinref part="SW2" gate="A" pin="1"/>
<pinref part="SUPPLY42" gate="G$1" pin="VCC"/>
<pinref part="SW2" gate="A" pin="3"/>
<pinref part="SW2" gate="A" pin="2"/>
<wire x1="81.28" y1="147.32" x2="81.28" y2="149.86" width="0.1524" layer="91"/>
<wire x1="81.28" y1="149.86" x2="81.28" y2="152.4" width="0.1524" layer="91"/>
<junction x="81.28" y="149.86"/>
<junction x="81.28" y="152.4"/>
</segment>
<segment>
<pinref part="SUPPLY46" gate="G$1" pin="VCC"/>
<pinref part="U1" gate="G$1" pin="VDD"/>
</segment>
<segment>
<pinref part="SUPPLY48" gate="G$1" pin="VCC"/>
<pinref part="U2" gate="G$1" pin="VDD"/>
</segment>
</net>
<net name="INT" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="4"/>
<wire x1="30.48" y1="38.1" x2="35.56" y2="38.1" width="0.1524" layer="91"/>
<label x="35.56" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="INTA"/>
<wire x1="119.38" y1="106.68" x2="121.92" y2="106.68" width="0.1524" layer="91"/>
<label x="121.92" y="106.68" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU22" class="0">
<segment>
<pinref part="SW2" gate="A" pin="4"/>
<wire x1="96.52" y1="147.32" x2="114.3" y2="147.32" width="0.1524" layer="91"/>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="114.3" y1="147.32" x2="127" y2="147.32" width="0.1524" layer="91"/>
<junction x="114.3" y="147.32"/>
<label x="127" y="147.32" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="A2"/>
<wire x1="88.9" y1="93.98" x2="86.36" y2="93.98" width="0.1524" layer="91"/>
<label x="73.66" y="93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU21" class="0">
<segment>
<pinref part="SW2" gate="A" pin="5"/>
<wire x1="96.52" y1="149.86" x2="106.68" y2="149.86" width="0.1524" layer="91"/>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="106.68" y1="149.86" x2="127" y2="149.86" width="0.1524" layer="91"/>
<junction x="106.68" y="149.86"/>
<label x="127" y="149.86" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="A1"/>
<wire x1="88.9" y1="96.52" x2="86.36" y2="96.52" width="0.1524" layer="91"/>
<label x="73.66" y="96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU20" class="0">
<segment>
<pinref part="SW2" gate="A" pin="6"/>
<wire x1="96.52" y1="152.4" x2="99.06" y2="152.4" width="0.1524" layer="91"/>
<pinref part="R6" gate="G$1" pin="1"/>
<wire x1="99.06" y1="152.4" x2="127" y2="152.4" width="0.1524" layer="91"/>
<junction x="99.06" y="152.4"/>
<label x="127" y="152.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U2" gate="G$1" pin="A0"/>
<wire x1="88.9" y1="99.06" x2="86.36" y2="99.06" width="0.1524" layer="91"/>
<label x="73.66" y="99.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU10" class="0">
<segment>
<pinref part="SW1" gate="A" pin="6"/>
<wire x1="27.94" y1="152.4" x2="30.48" y2="152.4" width="0.1524" layer="91"/>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="30.48" y1="152.4" x2="58.42" y2="152.4" width="0.1524" layer="91"/>
<junction x="30.48" y="152.4"/>
<label x="58.42" y="152.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="A0"/>
<wire x1="22.86" y1="99.06" x2="20.32" y2="99.06" width="0.1524" layer="91"/>
<label x="7.62" y="99.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU11" class="0">
<segment>
<pinref part="SW1" gate="A" pin="5"/>
<wire x1="27.94" y1="149.86" x2="38.1" y2="149.86" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="38.1" y1="149.86" x2="58.42" y2="149.86" width="0.1524" layer="91"/>
<junction x="38.1" y="149.86"/>
<label x="58.42" y="149.86" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="A1"/>
<wire x1="22.86" y1="96.52" x2="20.32" y2="96.52" width="0.1524" layer="91"/>
<label x="7.62" y="96.52" size="1.778" layer="95"/>
<label x="7.62" y="96.52" size="1.778" layer="95"/>
<label x="7.62" y="96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="ADDRU12" class="0">
<segment>
<pinref part="SW1" gate="A" pin="4"/>
<wire x1="27.94" y1="147.32" x2="45.72" y2="147.32" width="0.1524" layer="91"/>
<pinref part="R3" gate="G$1" pin="1"/>
<wire x1="45.72" y1="147.32" x2="58.42" y2="147.32" width="0.1524" layer="91"/>
<junction x="45.72" y="147.32"/>
<label x="58.42" y="147.32" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="A2"/>
<wire x1="22.86" y1="93.98" x2="20.32" y2="93.98" width="0.1524" layer="91"/>
<label x="7.62" y="93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="U2" gate="G$1" pin="!RESET"/>
<pinref part="RESET2" gate="P" pin="P"/>
<wire x1="86.36" y1="111.76" x2="88.9" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="!RESET"/>
<pinref part="RESET1" gate="P" pin="P"/>
<wire x1="20.32" y1="111.76" x2="22.86" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
<errors>
<approved hash="104,1,53.34,116.84,U1,VDD,VCC,,,"/>
<approved hash="104,1,53.34,66.04,U1,VSS,GND,,,"/>
<approved hash="104,1,119.38,116.84,U2,VDD,VCC,,,"/>
<approved hash="104,1,119.38,66.04,U2,VSS,GND,,,"/>
<approved hash="111,1,38.1,152.4,ADDRU10,,,,,"/>
<approved hash="111,1,45.72,149.86,ADDRU11,,,,,"/>
<approved hash="111,1,106.68,152.4,ADDRU20,,,,,"/>
<approved hash="111,1,114.3,149.86,ADDRU21,,,,,"/>
<approved hash="113,1,68.9017,44.45,SDA,,,,,"/>
<approved hash="113,1,76.5217,44.45,SCL,,,,,"/>
<approved hash="113,1,61.2817,44.45,VCC,,,,,"/>
<approved hash="113,1,130.071,89.431,FRAME1,,,,,"/>
<approved hash="113,1,20.32,148.364,SW1,,,,,"/>
<approved hash="113,1,88.9,148.364,SW2,,,,,"/>
<approved hash="113,1,86.4427,113.027,RESET2,,,,,"/>
<approved hash="113,1,20.4027,113.027,RESET1,,,,,"/>
</errors>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
<note version="9.5" severity="warning">
Since Version 9.5, EAGLE supports persistent groups with
schematics, and board files. Those persistent groups
will not be understood (or retained) with this version.
</note>
<note version="9.4" severity="warning">
Since Version 9.4, EAGLE supports the overriding of 3D packages
in schematics and board files. Those overridden 3d packages
will not be understood (or retained) with this version.
</note>
<note version="8.4" severity="warning">
Since Version 8.4, EAGLE supports properties for SPICE simulation. 
Probes in schematics and SPICE mapping objects found in parts and library devices
will not be understood with this version. Update EAGLE to the latest version
for full support of SPICE simulation. 
</note>
</compatibility>
</eagle>
