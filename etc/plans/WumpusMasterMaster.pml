<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1571827339498" name="WumpusMasterMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1571827339499" name="Wait4Start" comment="" entryPoint="1571827339500">
    <outTransitions>#1571833037764</outTransitions>
  </states>
  <states id="1571832997210" name="NewState" comment="">
    <plans xsi:type="alica:Plan">SingleAgent/WumpusMaster.pml#1534835261527</plans>
    <inTransitions>#1571833037764</inTransitions>
    <outTransitions>#1574260067168</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1574260058143" name="ExperimentComplete" comment="">
    <inTransitions>#1574260067168</inTransitions>
  </states>
  <transitions id="1571833037764" name="MISSING_NAME" comment="waited for start" msg="">
    <preCondition id="1571833039064" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1571827339499</inState>
    <outState>#1571832997210</outState>
  </transitions>
  <transitions id="1574260067168" name="MISSING_NAME" comment="Experiment Complete" msg="">
    <preCondition id="1574260068271" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1571832997210</inState>
    <outState>#1574260058143</outState>
  </transitions>
  <entryPoints id="1571827339500" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1571827339499</state>
  </entryPoints>
</alica:Plan>
