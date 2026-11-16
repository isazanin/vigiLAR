using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// CameraTopo - enquadra a câmera em vista de cima (planta baixa) sobre a casa.
///
/// COMO USAR
/// 1. Coloque este script na Main Camera.
/// 2. Arraste a CASA (EXPO_HOUSE) para o campo "Casa".
/// 3. Aperte Play (ou clique nos três pontinhos do componente > "Enquadrar agora").
///
/// A câmera vira ortográfica (sem perspectiva), olha reto para baixo e
/// ajusta posição e zoom para a casa caber inteira na tela.
/// </summary>
[ExecuteAlways]
[RequireComponent(typeof(Camera))]
public class CameraTopo : MonoBehaviour
{
    public GameObject casa;

    [Tooltip("Folga ao redor da casa. 1 = encostada, 1.1 = 10% de margem.")]
    public float margem = 1.1f;

    [Tooltip("Altura da câmera acima do ponto mais alto da casa.")]
    public float alturaAcima = 10f;

    [Tooltip("Ignora objetos soltos longe da casa (que deixam a casa pequena e fora do centro).")]
    public bool ignorarObjetosDistantes = true;

    [Tooltip("Fração de objetos descartada em cada ponta. 0.02 = 2%.")]
    [Range(0f, 0.2f)] public float percentil = 0.02f;

    void Start()
    {
        Enquadrar();
    }

    void OnEnable()
    {
        Enquadrar();
    }

    void OnValidate()
    {
        // Evita rodar durante a importação/compilação
        if (isActiveAndEnabled) Enquadrar();
    }

    [ContextMenu("Enquadrar agora")]
    public void Enquadrar()
    {
        if (casa == null) return;

        Renderer[] renderers = casa.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) return;

        Bounds limites = CalcularLimites(renderers);

        Camera cam = GetComponent<Camera>();
        cam.orthographic = true;

        // Reto para baixo; o topo da imagem aponta para +Z (como a vista Top da Scene)
        transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        transform.position = new Vector3(
            limites.center.x,
            limites.max.y + alturaAcima,
            limites.center.z);

        // Zoom para a casa caber na largura e na altura da imagem
        float metadeAltura = limites.extents.z;
        float metadeLargura = limites.extents.x / Mathf.Max(cam.aspect, 0.01f);
        cam.orthographicSize = Mathf.Max(metadeAltura, metadeLargura) * margem;

        cam.nearClipPlane = 0.3f;
        cam.farClipPlane = 1000f;
    }

    Bounds CalcularLimites(Renderer[] renderers)
    {
        Bounds todos = renderers[0].bounds;
        foreach (Renderer r in renderers) todos.Encapsulate(r.bounds);

        if (!ignorarObjetosDistantes || renderers.Length < 20) return todos;

        // Descarta os objetos cujo centro está nas pontas extremas (fora de lugar)
        var xs = new List<float>(renderers.Length);
        var zs = new List<float>(renderers.Length);
        foreach (Renderer r in renderers)
        {
            Vector3 c = r.bounds.center;
            xs.Add(c.x);
            zs.Add(c.z);
        }
        xs.Sort();
        zs.Sort();

        int baixo = Mathf.FloorToInt((renderers.Length - 1) * percentil);
        int alto = Mathf.CeilToInt((renderers.Length - 1) * (1f - percentil));
        float minX = xs[baixo], maxX = xs[alto];
        float minZ = zs[baixo], maxZ = zs[alto];

        bool primeiro = true;
        Bounds resultado = todos;
        foreach (Renderer r in renderers)
        {
            Vector3 c = r.bounds.center;
            if (c.x < minX || c.x > maxX || c.z < minZ || c.z > maxZ) continue;

            if (primeiro) { resultado = r.bounds; primeiro = false; }
            else resultado.Encapsulate(r.bounds);
        }

        return primeiro ? todos : resultado;
    }
}
