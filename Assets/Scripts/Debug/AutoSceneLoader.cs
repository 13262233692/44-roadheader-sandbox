using UnityEngine;
using RoadheaderSandbox.Debug;

namespace RoadheaderSandbox.Debug
{
    public class AutoSceneLoader : MonoBehaviour
    {
        private static AutoSceneLoader _instance;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void InitializeBeforeSceneLoad()
        {
            if (_instance != null) return;

            GameObject loaderObj = new GameObject("AutoSceneLoader");
            _instance = loaderObj.AddComponent<AutoSceneLoader>();
            DontDestroyOnLoad(loaderObj);
        }

        private void Awake()
        {
            Application.targetFrameRate = 60;
            Time.fixedDeltaTime = 0.001f;
            Time.maximumDeltaTime = 0.01f;

            Physics.defaultMaxDepenetrationVelocity = 10f;
            Physics.defaultContactOffset = 0.001f;
        }

        private void Start()
        {
            BuildScene();
        }

        private void BuildScene()
        {
            GameObject sceneBuilderObj = new GameObject("SceneBuilder");
            SceneBuilder sceneBuilder = sceneBuilderObj.AddComponent<SceneBuilder>();

            try
            {
                sceneBuilder.BuildCompleteScene();
                sceneBuilder.RunValidationChecks();
                Debug.Log("=== 场景自动构建完成 ===");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"场景构建失败: {e.Message}\n{e.StackTrace}");
            }
        }
    }
}
