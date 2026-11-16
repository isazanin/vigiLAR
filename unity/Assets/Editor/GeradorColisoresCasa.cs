using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Unity.AI.Navigation;

/// <summary>
/// Ferramenta de editor do vigiLAR.
/// Coloque este arquivo numa pasta chamada "Editor" dentro de Assets.
/// Depois: selecione a casa na Hierarchy e use o menu vigiLAR > Gerar colisores da casa.
/// Os colisores ficam salvos na cena (salve com Ctrl+S).
/// </summary>
public static class GeradorColisoresCasa
{
    // Mesmos valores padrão do RoboVigilar. Ajuste se necessário.
    const float AlturaMinimaMovel = 0.15f;
    const float FracaoAreaPiso = 0.3f;

    [MenuItem("vigiLAR/Gerar colisores da casa")]
    static void Gerar()
    {
        GameObject casa = Selection.activeGameObject;
        if (casa == null)
        {
            EditorUtility.DisplayDialog("vigiLAR",
                "Selecione a casa (EXPO_HOUSE) na Hierarchy antes de usar esta opção.", "OK");
            return;
        }

        Renderer[] renderers = casa.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
        {
            EditorUtility.DisplayDialog("vigiLAR",
                "O objeto selecionado não tem nenhuma malha (Renderer) dentro dele.", "OK");
            return;
        }

        Bounds limites = renderers[0].bounds;
        foreach (Renderer r in renderers) limites.Encapsulate(r.bounds);
        float areaCasa = limites.size.x * limites.size.z;

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Gerar colisores da casa");
        int grupo = Undo.GetCurrentGroup();

        int caixas = 0, malhas = 0, moveis = 0;

        try
        {
            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer r = renderers[i];

                if (i % 20 == 0)
                {
                    EditorUtility.DisplayProgressBar("vigiLAR",
                        "Criando colisores... " + i + "/" + renderers.Length,
                        (float)i / renderers.Length);
                }

                MeshFilter mf = r.GetComponent<MeshFilter>();
                if (mf == null || mf.sharedMesh == null) continue;

                Bounds b = r.bounds;
                bool alto = b.size.y > AlturaMinimaMovel;
                bool gigante = (b.size.x * b.size.z) > areaCasa * FracaoAreaPiso;

                // Cria o colisor se o objeto ainda não tiver um
                if (r.GetComponent<Collider>() == null)
                {
                    if (gigante)
                    {
                        MeshCollider mc = Undo.AddComponent<MeshCollider>(r.gameObject);
                        mc.sharedMesh = mf.sharedMesh;
                        malhas++;
                    }
                    else
                    {
                        BoxCollider bc = Undo.AddComponent<BoxCollider>(r.gameObject);
                        bc.center = mf.sharedMesh.bounds.center;
                        bc.size = mf.sharedMesh.bounds.size;
                        caixas++;
                    }
                }

                // Móveis e paredes viram obstáculo (Not Walkable) no NavMesh
                if (alto && !gigante)
                {
                    NavMeshModifier mod = r.GetComponent<NavMeshModifier>();
                    if (mod == null) mod = Undo.AddComponent<NavMeshModifier>(r.gameObject);
                    mod.overrideArea = true;
                    mod.area = 1; // 1 = Not Walkable
                    moveis++;
                }
            }
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }

        Undo.CollapseUndoOperations(grupo);
        EditorSceneManager.MarkSceneDirty(casa.scene);

        EditorUtility.DisplayDialog("vigiLAR",
            "Pronto!\n\n" +
            caixas + " caixas de colisão criadas\n" +
            malhas + " colisores de malha criados (piso/estruturas)\n" +
            moveis + " objetos marcados como obstáculo\n\n" +
            "Salve a cena com Ctrl+S. Para desfazer, use Ctrl+Z.",
            "OK");
    }
}
