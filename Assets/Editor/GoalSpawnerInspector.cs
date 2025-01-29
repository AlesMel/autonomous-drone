
using UnityEditor;
using UnityEngine;


[CustomEditor(typeof(Goal))]
public class GoalSpawnerInspector : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI(); // Draw the default inspector

        Goal goal = (Goal)target;

        if (GUILayout.Button("Random spawn"))
        {
            goal.ResetGoal();
        }
    }
}

