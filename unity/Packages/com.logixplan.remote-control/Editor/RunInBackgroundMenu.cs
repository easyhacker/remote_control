// Menu: IVI Dynamic → Remote Control → Enable Run In Background
// A remote-controlled robot must keep simulating while you type in the controller's terminal. With Player
// setting "Run In Background" off, Unity pauses Play mode whenever its window loses focus, heartbeats stop,
// and the controller drops the robot after a couple of seconds.

using UnityEditor;
using UnityEngine;

namespace RobotMarket.RemoteControl.Editor
{
    public static class RunInBackgroundMenu
    {
        const string MenuPath = "IVI Dynamic/Remote Control/Enable Run In Background";

        [MenuItem(MenuPath)]
        static void Enable()
        {
            PlayerSettings.runInBackground = true;
            AssetDatabase.SaveAssets();
            Debug.Log("[RemoteControl] Player setting 'Run In Background' enabled — Unity keeps running when it loses focus.");
        }

        [MenuItem(MenuPath, true)]
        static bool Validate()
        {
            Menu.SetChecked(MenuPath, PlayerSettings.runInBackground);
            return true;
        }
    }
}
