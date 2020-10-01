using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityTools;
using UnityTools.Algorithm;
using UnityTools.Common;
using UnityTools.Debuging;
using UnityTools.Debuging.EditorTool;

namespace UnityPaperModel
{
    [ExecuteInEditMode]
    public class DualGraphMono : MonoBehaviour
    {
        [SerializeField] protected int2 index;
        protected DualGraph dualGraph;
        protected VertexGraph graph;
        protected NewDualGraph dg;
        protected void Start()
        {
            this.graph = new VertexGraph();
            this.dg = new NewDualGraph();

            var mesh = this.GetComponent<MeshFilter>().mesh;
            for (var i = 0; i < mesh.triangles.Length; i += 3)
            {
                var v1 = mesh.triangles[i];
                var v2 = mesh.triangles[i+1];
                var v3 = mesh.triangles[i+2];

                var nv1 = this.AddVert(mesh.vertices[v1]);
                var nv2 = this.AddVert(mesh.vertices[v2]);
                var nv3 = this.AddVert(mesh.vertices[v3]);

                var e1 = this.graph.AddEdge(nv1, nv2) as VertexGraph.Edge;
                var e2 = this.graph.AddEdge(nv2, nv3) as VertexGraph.Edge;
                var e3 = this.graph.AddEdge(nv3, nv1) as VertexGraph.Edge;


                this.AddFace(e1, e2, e3);
            }

            // var str = "";

            // var eset = new HashSet<VertexGraph.Edge>();
            // foreach(var f in this.dg.Vertices)
            // {
            //     foreach(var e in f.Edges)
            //     {
            //         eset.Add(e);
            //         Debug.Log(e);
            //         str += " " + e;
            //     }
            // }

            // Debug.Log(eset.Count);
            // Debug.Log(str);

            // foreach(var face in this.dg.Vertices)
            // {
            //     var v1 = face.Edges[0];
            //     var v2 = face.Edges[1];
            //     var v3 = face.Edges[2];
            //     this.CheckFaceEdge(face, v1);
            //     this.CheckFaceEdge(face, v2);
            //     this.CheckFaceEdge(face, v3);
            // }

            LogTool.Log("v count " + this.graph.Vertices.Count());
            LogTool.Log("mesh v count " + mesh.vertices.Count());
        }

        protected void AddFace(VertexGraph.Edge e1, VertexGraph.Edge e2, VertexGraph.Edge e3)
        {                
            var face = this.dg.Factory.CreateVertex() as NewDualGraph.Face;
            face.AddEdge(e1);
            face.AddEdge(e2);
            face.AddEdge(e3);
            this.dg.Add(face);

            this.CheckFaceEdge(face, e1);
            this.CheckFaceEdge(face, e2);
            this.CheckFaceEdge(face, e3);

        }

        protected void CheckFaceEdge(NewDualGraph.Face face, VertexGraph.Edge e)
        {
            var other = this.dg.FaceContainsVertexOtherThan(face, e);
            if(other != default)
            {
                this.dg.AddEdge(face, other);
            }
        }

        protected VertexGraph.Vertex AddVert(float3 position)
        {
            var nv = this.graph.Factory.CreateVertex() as VertexGraph.Vertex;
            nv.Position = position;
            this.graph.Add(nv);
            return nv;
        }

        protected void OnDrawGizmos()
        {
            using(new GizmosScope(Color.cyan, this.transform.localToWorldMatrix))
            {
                this.dg?.OnDrawGizmos();
            }
        }
        // protected GraphAdj<DualGraph.Face, DualGraph.DualEdge> msp;
        // protected void Start()
        // {
        //     var mesh = this.GetComponent<MeshFilter>().mesh;
        //     this.dualGraph = new DualGraph(mesh);

        //     // this.dualGraph = DualGraph.Load();


        //     this.msp = this.BFS(this.dualGraph.GetDualGraph);

        //     var visited = new HashSet<DualGraph.Face>();
        //     this.UpdateLocal(this.msp, this.msp.Nodes.ToList()[3], visited, Matrix4x4.identity);
        // }
        // protected void Update()
        // {
        //     if (Input.GetKeyDown(KeyCode.R))
        //     {
        //         var edge = this.dualGraph.GetDualGraph.GetEdge(index.x, index.y);
        //         if (edge != null) this.dualGraph.GetDualGraph.RemoveEdge(edge);
        //     }
        //     if (Input.GetKeyDown(KeyCode.S)) DualGraph.Save(this.dualGraph);
        //     if (Input.GetKeyDown(KeyCode.L)) this.dualGraph = DualGraph.Load();
        // }

        // protected void OnDrawGizmos()
        // {
        //     this.dualGraph?.OnDrawGizmos();

        //     if (this.msp != null)
        //     {
        //         using (new GizmosScope(Color.cyan, this.transform.localToWorldMatrix))
        //         {
        //             foreach (var n in this.msp.Nodes)
        //             {
        //                 Gizmos.DrawSphere(n.Center, 0.01f);
        //             }

        //             foreach (var e in this.msp.Edges)
        //             {
        //                 Gizmos.DrawLine(e.Start.Center, e.End.Center);
        //             }
        //         }

        //         using (new GizmosScope(Color.green, this.transform.localToWorldMatrix))
        //         {
        //             foreach (var n in this.msp.Nodes)
        //             {
        //                 Gizmos.DrawLine(n.Center, n.Center + n.Normal);
        //             }
        //         }
        //         using (new GizmosScope(Color.blue, this.transform.localToWorldMatrix))
        //         {
        //             foreach (var n in this.msp.Nodes)
        //             {
        //                 Gizmos.DrawLine(n.world[0], n.world[1]);
        //                 Gizmos.DrawLine(n.world[1], n.world[2]);
        //                 Gizmos.DrawLine(n.world[2], n.world[0]);
        //                            }
        //         }
        //         using (new GizmosScope(Color.red, this.transform.localToWorldMatrix))
        //         {
        //             foreach (var n in this.msp.Nodes)
        //             {
                         
        //                 Gizmos.DrawLine(n.newWorld[0], n.newWorld[1]);
        //                 Gizmos.DrawLine(n.newWorld[1], n.newWorld[2]);
        //                 Gizmos.DrawLine(n.newWorld[2], n.newWorld[0]);
        //             }
        //         }
 
        //     }
        // }


        // protected GraphAdj<DualGraph.Face, DualGraph.DualEdge> BFS(IGraph<DualGraph.Face, DualGraph.DualEdge> graph)
        // {
        //     var gnodes = graph.Nodes.ToList();
        //     var ret = new GraphAdj<DualGraph.Face, DualGraph.DualEdge>(gnodes.Count);
        //     var nnodes = ret.Nodes.ToList();
        //     var queue = new Queue<DualGraph.Face>();
        //     var visitied = new HashSet<DualGraph.Face>();

        //     queue.Enqueue(gnodes.First());


        //     while (queue.Count > 0)
        //     {
        //         var node = queue.Dequeue();
        //         nnodes[node.Index].Center = node.Center;
        //         nnodes[node.Index].world = node.world;
        //         nnodes[node.Index].newWorld = node.newWorld;

        //         foreach (var n in graph.GetNeighborsNodes(node))
        //         {
        //             if (visitied.Contains(n)) continue;

        //             queue.Enqueue(n);
        //             visitied.Add(n);

        //             var e = new DualGraph.DualEdge();

        //             var oe = graph.GetEdge(node, n);
        //             e.edge = oe.edge;

        //             ret.AddEdge(node, n, e);
        //         }

        //         if (queue.Count == 0 && visitied.Count < gnodes.Count)
        //         {
        //             foreach (var n in gnodes)
        //             {
        //                 if (visitied.Contains(n) == false)
        //                 {
        //                     queue.Enqueue(n);
        //                     break;
        //                 }
        //             }
        //         }
        //     }

        //     return ret;
        // }

        // public class TR
        // {
        //     public Matrix4x4 trans;
        //     public Matrix4x4 rot;
        // }

        // protected void UpdateLocal(GraphAdj<DualGraph.Face, DualGraph.DualEdge> graph, DualGraph.Face node, HashSet<DualGraph.Face> visited, Matrix4x4 parent)
        // {
        //     if (visited.Contains(node)) return;
        //     visited.Add(node);

        //     for(var i = 0; i< node.newWorld.Count; ++i)
        //     {
        //         node.newWorld[i] = parent.MultiplyPoint(node.world[i]);
        //     }

        //     foreach (var e in graph.GetNeighborsEdges(node))
        //     {
        //         LogTool.AssertIsTrue(e.Start == node);
        //         var org = e.edge.Start.position;
        //         var axis = e.edge.Start.position - e.edge.End.position;
        //         var n1 = e.Start.Normal;
        //         var n2 = e.End.Normal;
        //         var cos = math.dot(n1, n2);
        //         var angle = math.acos(cos) * Mathf.Rad2Deg;

        //         var dir = math.normalize(e.Start.Center - e.End.Center);
        //         var dirAngle = math.dot(n1, dir);

        //         if (dirAngle > 0) angle = 360 - angle;
        //         var mat = Matrix4x4.Rotate(Quaternion.AngleAxis(angle, axis));

        //         e.End.local = Matrix4x4.Translate(-org) * mat * Matrix4x4.Translate(org);

        //         this.UpdateLocal(graph, e.End, visited, parent * e.End.local);
        //     }

        // }

        // protected int[] GetVertexMap(List<float3> node, List<float3> next)
        // {
        //     var ret = new int[node.Count];
        //     ret[0] = next.Select((pos, i) => new { pos, i }).First(it => math.distance(it.pos, node[0]) < 0.001f).i;
        //     ret[1] = next.Select((pos, i) => new { pos, i }).First(it => math.distance(it.pos, node[1]) < 0.001f).i;
        //     ret[2] = next.Select((pos, i) => new { pos, i }).First(it => math.distance(it.pos, node[2]) < 0.001f).i;
        //     return ret;
        // }

        


        // protected void FlatGraph(GraphAdj<DualGraph.Face, DualGraph.DualEdge> graph, DualGraph.Face node, HashSet<DualGraph.Face> visited)
        // {
        //     if (visited.Contains(node)) return;
        //     visited.Add(node);

        //     this.MoveToUp(node);

        //     foreach (var next in graph.GetNeighborsNodes(node))
        //     {
        //         // this.MoveToUp(next);
        //          this.FlatGraph(graph, next, visited);
        //     }

            
        // }

        // protected void MoveToUp(DualGraph.Face node)
        // {
        //     var up = new float3(0,1,0);
        //     var n = node.Normal;
        //     var angle = math.acos(math.dot(up, n)) * Mathf.Rad2Deg;

        //     var axis = math.cross(up, n);
        //     var mat = Matrix4x4.Rotate(Quaternion.AngleAxis(360-angle, axis));

        //     // node.Rotate(node.Center, mat);


        // }

        // protected DualGraph.Face GetFaceFrom(DualGraph.Face node, List<TR> trans)
        // {
        //     var ret = new DualGraph.Face();
        //     ret.Init(node.world.DeepCopy());

        //     foreach(var mat in trans)
        //     {
        //         // node.Rotate(mat.trans, mat.rot);
        //     }

        //     return ret;
        // }

        // protected void UpdateNeighborsVertex(GraphAdj<DualGraph.Face, DualGraph.DualEdge> graph, DualGraph.Face node, float3 v1, float3 v2, float3 v3)
        // {
        // }

    }
}