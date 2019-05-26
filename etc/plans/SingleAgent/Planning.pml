<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1551262808911" name="Planning" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/SingleAgent" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1551262808912" name="DetermineGoal" comment="" entryPoint="1551262808913">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DetermineGoal.beh#1551695076041</plans>
    <inTransitions>#1551263164133</inTransitions>
    <outTransitions>#1551263120298</outTransitions>
  </states>
  <states id="1551262842117" name="EvaluateGoal" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/EvaluateGoal.beh#1551695098593</plans>
    <inTransitions>#1551263120298</inTransitions>
    <inTransitions>#1551263668006</inTransitions>
    <outTransitions>#1551263159876</outTransitions>
    <outTransitions>#1551263164133</outTransitions>
  </states>
  <states id="1551263139139" name="PlanActions" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/GenerateActions.beh#1551695113275</plans>
    <inTransitions>#1551263159876</inTransitions>
    <inTransitions>#1551695257323</inTransitions>
    <outTransitions>#1551263271743</outTransitions>
  </states>
  <states id="1551263260517" name="EvaluateActions" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/EvaluateActions.beh#1551695131246</plans>
    <inTransitions>#1551263271743</inTransitions>
    <inTransitions>#1551263629562</inTransitions>
    <outTransitions>#1551263628215</outTransitions>
    <outTransitions>#1551263668006</outTransitions>
    <outTransitions>#1551695257323</outTransitions>
  </states>
  <states id="1551263346551" name="Perform Action" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/PerformNextAction.beh#1551695161214</plans>
    <inTransitions>#1551263628215</inTransitions>
    <outTransitions>#1551263629562</outTransitions>
  </states>
  <transitions id="1551263120298" name="MISSING_NAME" comment="goal found" msg="">
    <preCondition id="1551263121075" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551262808912</inState>
    <outState>#1551262842117</outState>
  </transitions>
  <transitions id="1551263159876" name="MISSING_NAME" comment="goal valid" msg="">
    <preCondition id="1551263161484" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551262842117</inState>
    <outState>#1551263139139</outState>
  </transitions>
  <transitions id="1551263164133" name="MISSING_NAME" comment="goal not safely reachable &amp;&amp; other fields safely reachable || haveGold &amp;&amp; goal not start field || glitter known &amp;&amp; goal not gold field" msg="">
    <preCondition id="1551263165293" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551262842117</inState>
    <outState>#1551262808912</outState>
  </transitions>
  <transitions id="1551263271743" name="MISSING_NAME" comment="found action sequence to reach goal" msg="">
    <preCondition id="1551263272837" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551263139139</inState>
    <outState>#1551263260517</outState>
  </transitions>
  <transitions id="1551263628215" name="MISSING_NAME" comment="actions valid" msg="">
    <preCondition id="1551263629178" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551263260517</inState>
    <outState>#1551263346551</outState>
  </transitions>
  <transitions id="1551263629562" name="MISSING_NAME" comment="received new perception" msg="">
    <preCondition id="1551263634437" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551263346551</inState>
    <outState>#1551263260517</outState>
  </transitions>
  <transitions id="1551263668006" name="MISSING_NAME" comment="actions invalid &amp;&amp; goal invalid" msg="">
    <preCondition id="1551263669644" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551263260517</inState>
    <outState>#1551262842117</outState>
  </transitions>
  <transitions id="1551695257323" name="MISSING_NAME" comment="goal valid &amp;&amp; actions invalid" msg="">
    <preCondition id="1551695258558" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1551263260517</inState>
    <outState>#1551263139139</outState>
  </transitions>
  <entryPoints id="1551262808913" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1551262808912</state>
  </entryPoints>
</alica:Plan>
