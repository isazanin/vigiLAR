using System.Collections;
using UnityEngine;
using UnityEngine.AI;
using Unity.AI.Navigation;

/// <summary>
/// Coloque este script no ROBÔ. Arraste a CASA para o campo "Casa".
/// Ao apertar Play ele: gera o NavMesh da casa, configura o NavMeshAgent,
/// faz o robô patrulhar pontos aleatórios da casa e gira as rodas.
/// Requer o pacote "AI Navigation" (Window > Package Manager).
/// </summary>
public class RoboAutonomo : MonoBehaviour
{
    [Header("Arraste a casa aqui")]
    public GameObject casa;

    [Header("Movimento")]
    public float velocidade = 1.5f;
    public float velocidadeGiro = 180f;
    public float raioRobo = 0.3f;
    public float alturaRobo = 0.8f;
    public float tempoParadaNoPonto = 1f;

    [Header("Rodas (opcional)")]
    [Tooltip("Se vazio, procura filhos com 'roda' ou 'wheel' no nome.")]
    public Transform[] rodas;
    public float raioRoda = 0.1f;
    [Tooltip("Eixo de giro das rodas. Se girar errado, troque (X, Y ou Z).")]
    public Vector3 eixoRoda = Vector3.right;

    private NavMeshAgent agente;
    private Bounds limitesCasa;

    void Start()
    {
        if (casa == null)
        {
            Debug.LogError("RoboAutonomo: arraste a casa para o campo 'Casa'.");
            enabled = false;
            return;
        }

        // 1) Gera o NavMesh da casa (se ainda não existir)
        NavMeshSurface superficie = casa.GetComponent<NavMeshSurface>();
        if (superficie == null)
        {
            superficie = casa.AddComponent<NavMeshSurface>();
            superficie.collectObjects = CollectObjects.Children;
            superficie.useGeometry = NavMeshCollectGeometry.RenderMeshes;
            superficie.BuildNavMesh();
        }

        // 2) Limites da casa, para sortear pontos de ronda
        Renderer[] renderers = casa.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
        {
            Debug.LogError("RoboAutonomo: a casa não tem nenhum Renderer.");
            enabled = false;
            return;
        }
        limitesCasa = renderers[0].bounds;
        foreach (Renderer r in renderers) limitesCasa.Encapsulate(r.bounds);

        // 3) Configura o agente
        agente = GetComponent<NavMeshAgent>();
        if (agente == null) agente = gameObject.AddComponent<NavMeshAgent>();
        agente.enabled = false; // evita erro se o robô começar fora do NavMesh
        agente.speed = velocidade;
        agente.angularSpeed = velocidadeGiro;
        agente.radius = raioRobo;
        agente.height = alturaRobo;
        agente.acceleration = 4f;
        agente.stoppingDistance = 0.3f;

        // 4) Coloca o robô no ponto navegável mais próximo
        if (NavMesh.SamplePosition(transform.position, out NavMeshHit hit, 10f, NavMesh.AllAreas))
        {
            transform.position = hit.position;
            agente.enabled = true;
        }
        else
        {
            Debug.LogError("RoboAutonomo: não achei NavMesh perto do robô. Coloque o robô dentro da casa, no chão.");
            enabled = false;
            return;
        }

        // 5) Acha as rodas automaticamente
        if (rodas == null || rodas.Length == 0)
        {
            var lista = new System.Collections.Generic.List<Transform>();
            foreach (Transform t in GetComponentsInChildren<Transform>())
            {
                string nome = t.name.ToLower();
                if (nome.Contains("roda") || nome.Contains("wheel")) lista.Add(t);
            }
            rodas = lista.ToArray();
        }

        StartCoroutine(Patrulha());
    }

    void Update()
    {
        if (agente == null || !agente.enabled || rodas == null) return;

        float graus = (agente.velocity.magnitude * Time.deltaTime / raioRoda) * Mathf.Rad2Deg;
        foreach (Transform roda in rodas)
        {
            if (roda != null) roda.Rotate(eixoRoda, graus, Space.Self);
        }
    }

    IEnumerator Patrulha()
    {
        yield return null;
        while (true)
        {
            if (EscolherPonto(out Vector3 destino))
            {
                agente.SetDestination(destino);

                float tempo = 0f;
                while (tempo < 60f) // desiste do ponto se demorar demais
                {
                    tempo += Time.deltaTime;
                    yield return null;
                    if (agente.pathPending) continue;
                    if (agente.pathStatus == NavMeshPathStatus.PathInvalid) break;
                    if (agente.remainingDistance <= agente.stoppingDistance + 0.1f) break;
                }
                yield return new WaitForSeconds(tempoParadaNoPonto);
            }
            else
            {
                yield return new WaitForSeconds(0.5f);
            }
        }
    }

    bool EscolherPonto(out Vector3 destino)
    {
        destino = transform.position;
        NavMeshPath caminho = new NavMeshPath();

        for (int i = 0; i < 30; i++)
        {
            Vector3 sorteio = new Vector3(
                Random.Range(limitesCasa.min.x, limitesCasa.max.x),
                limitesCasa.min.y + 0.5f,
                Random.Range(limitesCasa.min.z, limitesCasa.max.z));

            if (NavMesh.SamplePosition(sorteio, out NavMeshHit hit, 3f, NavMesh.AllAreas))
            {
                if (agente.CalculatePath(hit.position, caminho) &&
                    caminho.status == NavMeshPathStatus.PathComplete)
                {
                    destino = hit.position;
                    return true;
                }
            }
        }
        return false;
    }
}
