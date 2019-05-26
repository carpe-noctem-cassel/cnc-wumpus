<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1554202530131" name="NextAction" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1554202530132" name="DetermineObjective" comment="" entryPoint="1554202530133">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DetermineObjective.beh#1554202577790</plans>
    <outTransitions>#1554202970601</outTransitions>
  </states>
  <states id="1554202945017" name="PlanAndExecute" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/PerformAction.beh#1534836780649</plans>
    <inTransitions>#1554202970601</inTransitions>
    <outTransitions>#1554203053575</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1554202959483" name="Success" comment="">
    <inTransitions>#1554203053575</inTransitions>
  </states>
  <transitions id="1554202970601" name="MISSING_NAME" comment="determine obj success" msg="">
    <preCondition id="1554202971930" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1554202530132</inState>
    <outState>#1554202945017</outState>
  </transitions>
  <transitions id="1554203053575" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1554203054731" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1554202945017</inState>
    <outState>#1554202959483</outState>
  </transitions>
  <entryPoints id="1554202530133" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1554202530132</state>
  </entryPoints>
</alica:Plan>
