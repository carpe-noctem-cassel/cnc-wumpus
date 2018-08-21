<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1534835261527" name="WumpusMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1534835261528" name="Stop" comment="" entryPoint="1534835261530">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/SpawnAgent.beh#1534835358495</plans>
    <outTransitions>#1534836323236</outTransitions>
  </states>
  <states id="1534836316488" name="Play" comment="">
    <plans xsi:type="alica:Plan">InteractWithPlayground.pml#1534836538908</plans>
    <inTransitions>#1534836323236</inTransitions>
    <outTransitions>#1534836414368</outTransitions>
    <outTransitions>#1534836446000</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1534836405246" name="AgentLeftCave" comment="">
    <inTransitions>#1534836414368</inTransitions>
  </states>
  <states xsi:type="alica:FailureState" id="1534836429671" name="AgentDead" comment="">
    <inTransitions>#1534836446000</inTransitions>
  </states>
  <transitions id="1534836323236" name="MISSING_NAME" comment="spawnAgent success" msg="">
    <preCondition id="1534836324356" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534835261528</inState>
    <outState>#1534836316488</outState>
  </transitions>
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
  <entryPoints id="1534835261530" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1534835261528</state>
  </entryPoints>
</alica:Plan>
