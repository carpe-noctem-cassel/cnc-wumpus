<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1534835261527" name="WumpusMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1536063740426" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1536152982013</vars>
  </conditions>
  <vars id="1536152982013" name="ModelVar" comment="" Type=""/>
  <states id="1534835261528" name="Wait" comment="" entryPoint="1534835261530">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/WaitForTurn.beh#1534835374750</plans>
    <inTransitions>#1574259998646</inTransitions>
    <outTransitions>#1537958014769</outTransitions>
  </states>
  <states id="1534836316488" name="Play" comment="">
    <plans xsi:type="alica:Plan">InteractWithPlayground.pml#1534836538908</plans>
    <inTransitions>#1537958014769</inTransitions>
    <outTransitions>#1574259996673</outTransitions>
  </states>
  <states id="1571748613432" name="SpawnAgents" comment="" entryPoint="1571748599471">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/SpawnAgent.beh#1534835358495</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/LogPreviousResults.beh#1575467908745</plans>
    <inTransitions>#1574260001687</inTransitions>
    <outTransitions>#1572431679710</outTransitions>
  </states>
  <states id="1572431590132" name="Play" comment="">
    <plans xsi:type="alica:Plan">InteractWithPlayground.pml#1534836538908</plans>
    <inTransitions>#1572431679710</inTransitions>
    <outTransitions>#1574259999778</outTransitions>
  </states>
  <states id="1574259961919" name="Reset" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Reset.beh#1572878624935</plans>
    <inTransitions>#1574259996673</inTransitions>
    <outTransitions>#1574259998646</outTransitions>
  </states>
  <states id="1574259968198" name="Reset" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Reset.beh#1572878624935</plans>
    <inTransitions>#1574259999778</inTransitions>
    <outTransitions>#1574260001687</outTransitions>
  </states>
  <transitions id="1537958014769" name="MISSING_NAME" comment="received initial pose response" msg="">
    <preCondition id="1537958016407" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534835261528</inState>
    <outState>#1534836316488</outState>
  </transitions>
  <transitions id="1572431679710" name="MISSING_NAME" comment="received initial pose response" msg="">
    <preCondition id="1572431680595" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1571748613432</inState>
    <outState>#1572431590132</outState>
  </transitions>
  <transitions id="1574259996673" name="MISSING_NAME" comment="exited" msg="">
    <preCondition id="1574259998405" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836316488</inState>
    <outState>#1574259961919</outState>
  </transitions>
  <transitions id="1574259998646" name="MISSING_NAME" comment="resetted" msg="">
    <preCondition id="1574259999562" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1574259961919</inState>
    <outState>#1534835261528</outState>
  </transitions>
  <transitions id="1574259999778" name="MISSING_NAME" comment="agent spawner exited" msg="">
    <preCondition id="1574260001551" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1572431590132</inState>
    <outState>#1574259968198</outState>
  </transitions>
  <transitions id="1574260001687" name="MISSING_NAME" comment="all agents exited spawner" msg="">
    <preCondition id="1574260002562" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1574259968198</inState>
    <outState>#1571748613432</outState>
  </transitions>
  <entryPoints id="1534835261530" name="CompleteRun" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1571831044421</task>
    <state>#1534835261528</state>
  </entryPoints>
  <entryPoints id="1571748599471" name="SpawnAgents" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1571748498123</task>
    <state>#1571748613432</state>
  </entryPoints>
</alica:Plan>
