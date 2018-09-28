<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1534835261527" name="WumpusMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="1" maxCardinality="1">
  <conditions xsi:type="alica:RuntimeCondition" id="1536063740426" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1536152982013</vars>
  </conditions>
  <vars id="1536152982013" name="ModelVar" comment="" Type=""/>
  <states id="1534835261528" name="Stop" comment="" entryPoint="1534835261530">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/SpawnAgent.beh#1534835358495</plans>
    <outTransitions>#1537958014769</outTransitions>
  </states>
  <states id="1534836316488" name="Play" comment="">
    <plans xsi:type="alica:Plan">InteractWithPlayground.pml#1534836538908</plans>
    <inTransitions>#1537958014769</inTransitions>
    <outTransitions>#1534836414368</outTransitions>
    <outTransitions>#1534836446000</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1534836405246" name="AgentLeftCave" comment="">
    <inTransitions>#1534836414368</inTransitions>
  </states>
  <states xsi:type="alica:FailureState" id="1534836429671" name="AgentDead" comment="">
    <inTransitions>#1534836446000</inTransitions>
  </states>
  <transitions id="1534836414368" name="MISSING_NAME" comment="exited response received" msg="">
    <preCondition id="1534836415069" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836316488</inState>
    <outState>#1534836405246</outState>
  </transitions>
  <transitions id="1534836446000" name="MISSING_NAME" comment="dead response received" msg="">
    <preCondition id="1534836447768" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836316488</inState>
    <outState>#1534836429671</outState>
  </transitions>
  <transitions id="1537958014769" name="MISSING_NAME" comment="spawned agent" msg="">
    <preCondition id="1537958016407" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534835261528</inState>
    <outState>#1534836316488</outState>
  </transitions>
  <entryPoints id="1534835261530" name="MISSING_NAME" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1534835261528</state>
  </entryPoints>
</alica:Plan>
