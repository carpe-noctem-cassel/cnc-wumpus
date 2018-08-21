<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1534836685239" name="MyTurn" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1534836698773" name="PerformAction" comment="" entryPoint="1534836698774">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/PerformAction.beh#1534836780649</plans>
    <outTransitions>#1534836736600</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1534836725909" name="ActionPerformed" comment="">
    <inTransitions>#1534836736600</inTransitions>
  </states>
  <transitions id="1534836736600" name="MISSING_NAME" comment="any child success " msg="">
    <preCondition id="1534836737992" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836698773</inState>
    <outState>#1534836725909</outState>
  </transitions>
  <entryPoints id="1534836698774" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1534836698773</state>
  </entryPoints>
</alica:Plan>
