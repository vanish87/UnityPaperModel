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
    using VertexCombine = Dictionary<VertexGraph.CloseVertex, VertexGraph.Vertex>;
    public class DualGraphMono : MonoBehaviour
    {
        [SerializeField] protected int2 index;
        protected VertexGraph graph;
        protected DualGraph dualGraph;
        protected DualGraph mst;
        protected DualGraph orgmst;

        public List<DualGraph.Face> faces;
        public List<DualGraph.Edge> edges;

        public List<(DualGraph.Face, DualGraph.Face)> overlap = new List<(DualGraph.Face, DualGraph.Face)>();
        public HashSet<Path> paths = new HashSet<Path>();
        public List<DualGraph> noOverlap = new List<DualGraph>();

        public HalfEdgeTest.HGraph hgraph;
        protected void Start()
        {
            this.graph = new VertexGraph();
            this.dualGraph = new DualGraph();

            this.hgraph = new HalfEdgeTest.HGraph();


            // this.AddTest();
            this.AddMesh();

            var map = new Dictionary<IVertex, IVertex>();
            foreach(var v in this.graph.Vertices)
            {
                var hv = this.hgraph.Factory.CreateVertex() as HalfEdgeTest.HGraph.V;
                hv.Position = v.Position;
                this.hgraph.AddVertex(hv);

                if (map.ContainsKey(v)) continue;
                map.Add(v, hv);
            }

            foreach(var e in this.graph.Edges)
            {
                var v1 = map[e.Vertex];
                var v2 = map[e.OtherVertex];
                this.hgraph.AddEdge(v1,v2);
            }

            var count = this.graph.Count;
            ProgressiveMesh.CalculateVerticeWeight(this.graph);
            //while (this.graph.Count > count * 0.5f) ProgressiveMesh.RemoveMinCostVertex(this.graph);
            // while (this.graph.Count > 100) ProgressiveMesh.RemoveMinCostVertex(this.graph);

            this.dualGraph.UpdateWeight(new float3(0, 1, 0), 0);
            this.mst = MinimumSpanningTree.KruskalMST(this.dualGraph) as DualGraph;
            this.orgmst = MinimumSpanningTree.Generate(this.dualGraph) as DualGraph;
            LogTool.Log("dual count " + this.dualGraph.Vertices.Count());
            LogTool.Log("mst count " + this.mst.Vertices.Count());
            LogTool.Log("mst e count " + this.mst.Edges.Count());


            LogTool.Log("dual circle " + GraphTools.HasCircle(this.dualGraph).ToString());
            LogTool.Log("mst circle " + GraphTools.HasCircle(this.mst).ToString());



            var start = this.mst.First() as DualGraph.Face;
            var up = new float3(0,1,0);
            var axis = math.normalize(math.cross(start.Normal, up));
            var angle = math.acos(math.dot(up, start.Normal));
            var parent = Matrix4x4.Rotate(Quaternion.AngleAxis(angle*Mathf.Rad2Deg, axis));

            DualGraph.CalculateLocalMatrix(this.mst, start, new HashSet<IVertex>());
            DualGraph.FlatGraph(this.mst, start, new HashSet<IVertex>(), parent);

            this.mst.MoveToYZero();
            this.overlap = this.mst.GetOverlapFace();

            this.faces = this.mst.Vertices.ToList();
            this.edges = this.mst.Edges.ToList();


            this.GeneratePathSet();
            this.GenerateMinimumSetCover();


            // Debug.Log(ProgressiveMesh.debug.Count);


        }

        protected void Update()
        {
            if(Input.GetKeyDown(KeyCode.C))
            {
                ProgressiveMesh.RemoveMinCostVertex(this.graph);
            }
        }

        protected void GeneratePathSet()
        {
            foreach(var (from, to) in this.overlap)
            {
                var path = GraphTools.GetPath(this.mst, from, to) as Path;
                if(path != null) this.paths.Add(path);
            }
        }

        protected void GenerateMinimumSetCover()
        {
            var universe = new HashSet<MinimumSetCover.ICost>();
            var from = new HashSet<ISet<MinimumSetCover.ICost>>();
            foreach(var p in this.paths)
            {
                var edges = new HashSet<MinimumSetCover.ICost>();
                for(var i = 1; i < p.Count; ++i)
                {
                    var edge = this.mst.GetEdge(p[i-1], p[i]);
                    edges.Add(edge as MinimumSetCover.ICost);
                    universe.Add(edge as MinimumSetCover.ICost);
                }
                var set = new HashSet<MinimumSetCover.ICost>(edges);
                from.Add(set);
            }
            var ret = new HashSet<IEdge>();

            var pathSet = MinimumSetCover.GetMinimunSetCover(universe, from, 
            (u, i, cost)=>
            {                    
                var candidateEdge = default(MinimumSetCover.ICost);
                var candidateMinEdge = default(MinimumSetCover.ICost);
                var candidate = default(ISet<MinimumSetCover.ICost>);
                foreach (var orderedPath in cost.OrderBy(c=>c.Value/c.Key.Count))
                {
                    var p = orderedPath.Key;
                    // if(p.IsSubsetOf(i)) continue;

                    foreach (var minEdge in i.OrderBy(e => e.Cost))
                    {
                        if(p.Contains(minEdge) && ret.Contains(minEdge as IEdge))
                        {
                            candidateEdge = minEdge;
                            break;
                        }
                    }
                    if(candidateEdge != default)
                    {
                        candidate = p;
                        break;
                    } 

                    var localMin = p.OrderBy(pe => pe.Cost).First();
                    if (candidateMinEdge == default || localMin.Cost < candidateMinEdge.Cost)
                    {
                        candidateMinEdge = localMin;
                        candidate = p;
                    }
                }
                
                if(candidateEdge != default)
                {
                    // ret.Add(candidateEdge as IEdge);
                }
                else
                if(candidateMinEdge != default)
                {
                    ret.Add(candidateMinEdge as IEdge);
                }

                LogTool.AssertNotNull(candidate);

                return candidate;
            }
            );


            foreach(var e in ret)
            {
                LogTool.Log(e.ToString());
                this.mst.Remove(e);
            }

            foreach(var e in ret)
            {
                var ng = MinimumSpanningTree.Generate(this.mst, e.Vertex);
                var same = false;
                foreach (var g in this.noOverlap)
                {
                    if (g.SetEquals(ng))
                    {
                        same = true;
                        break;
                    }
                }
                if(!same)this.noOverlap.Add(ng as DualGraph);

                same = false;
                ng = MinimumSpanningTree.Generate(this.mst, e.OtherVertex);
                foreach (var g in this.noOverlap)
                {
                    if (g.SetEquals(ng))
                    {
                        same = true;
                        break;
                    }
                }
                if(!same)this.noOverlap.Add(ng as DualGraph);
            }

            foreach(var g in this.noOverlap)
            {
                var bound = new Bounds();
                foreach(var v in g.Vertices)
                {
                    bound.Encapsulate(v.Center);
                }
                foreach(var v in g.Vertices)
                {
                    v.Transform(Matrix4x4.Translate(-bound.center));
                }

            }

            
        }

        protected VertexGraph.Vertex GetVertex(VertexCombine g, float3 position)
        {
            var nv = new VertexGraph.CloseVertex();
            nv.Position = position;
            if (g.ContainsKey(nv)) return g[nv];

            var nmv = new VertexGraph.Vertex();
            nmv.Position = position;
            g.Add(nv, nmv);

            return nmv;
        }

        public static Dictionary<(IVertex, IVertex, IVertex), float3> normals = new Dictionary<(IVertex, IVertex, IVertex), float3>();
        protected void AddMesh()
        {
            normals.Clear();
            var combined = new VertexCombine();
            var mesh = this.GetComponent<MeshFilter>().sharedMesh;
            mesh.Optimize();
            for (var i = 0; i < mesh.triangles.Length; i += 3)
            {
                var v1 = mesh.triangles[i];
                var v2 = mesh.triangles[i + 1];
                var v3 = mesh.triangles[i + 2];

                var nv1 = this.AddVert(this.GetVertex(combined, mesh.vertices[v1]));
                var nv2 = this.AddVert(this.GetVertex(combined, mesh.vertices[v2]));
                var nv3 = this.AddVert(this.GetVertex(combined, mesh.vertices[v3]));

                var e1 = this.graph.AddEdge(nv1, nv2) as VertexGraph.Edge;
                var e2 = this.graph.AddEdge(nv2, nv3) as VertexGraph.Edge;
                var e3 = this.graph.AddEdge(nv3, nv1) as VertexGraph.Edge;

                var normal = math.normalize(math.cross(nv2.Position - nv1.Position, nv3.Position - nv1.Position));

                normals.Add((nv1,nv2,nv3), normal);

                var f = this.AddFace(e1, e2, e3, normal);


                nv1.Face.Add(f);
                nv2.Face.Add(f);
                nv3.Face.Add(f);
            }

            LogTool.Log("graph v count " + this.graph.Vertices.Count());
            LogTool.Log("graph e count " + this.graph.Edges.Count());
            LogTool.Log("mesh v count " + mesh.vertices.Count());
        }

        protected void AddTest()
        {
            var combined = new VertexCombine();
            var nv1 = this.AddVert(this.GetVertex(combined, new float3(0, 0, 0)));
            var nv2 = this.AddVert(this.GetVertex(combined, new float3(1, 0, 1)));
            var nv3 = this.AddVert(this.GetVertex(combined, new float3(1, 0, 0)));

            var e1 = this.graph.AddEdge(nv1, nv2) as VertexGraph.Edge;
            var e2 = this.graph.AddEdge(nv2, nv3) as VertexGraph.Edge;
            var e3 = this.graph.AddEdge(nv3, nv1) as VertexGraph.Edge;

            var normal = new float3(0, 1, 0);


            this.AddFace(e1, e2, e3, normal);

            nv1 = this.AddVert(this.GetVertex(combined, new float3(0, 0, 0)));
            nv2 = this.AddVert(this.GetVertex(combined, new float3(1, 0, 0)));
            nv3 = this.AddVert(this.GetVertex(combined, new float3(1, 1, 0)));

            e1 = this.graph.AddEdge(nv1, nv2) as VertexGraph.Edge;
            e2 = this.graph.AddEdge(nv2, nv3) as VertexGraph.Edge;
            e3 = this.graph.AddEdge(nv3, nv1) as VertexGraph.Edge;



            normal = math.normalize(math.cross(nv2.Position - nv1.Position, nv3.Position - nv1.Position));
            this.AddFace(e1, e2, e3, normal);

            nv1 = this.AddVert(this.GetVertex(combined, new float3(1, 0, 0)));
            nv2 = this.AddVert(this.GetVertex(combined, new float3(2, 1, 0)));
            nv3 = this.AddVert(this.GetVertex(combined, new float3(1, 1, 0)));

            e1 = this.graph.AddEdge(nv1, nv2) as VertexGraph.Edge;
            e2 = this.graph.AddEdge(nv2, nv3) as VertexGraph.Edge;
            e3 = this.graph.AddEdge(nv3, nv1) as VertexGraph.Edge;


            normal = math.normalize(math.cross(nv2.Position - nv1.Position, nv3.Position - nv1.Position));
            this.AddFace(e1, e2, e3, normal);

        }

        protected IVertex AddFace(VertexGraph.Edge e1, VertexGraph.Edge e2, VertexGraph.Edge e3, float3 normal)
        {
            var face = this.dualGraph.Factory.CreateVertex() as DualGraph.Face;
            face.AddEdge(e1);
            face.AddEdge(e2);
            face.AddEdge(e3);
            face.Normal = normal;

            if (this.dualGraph.Contains(face))
            {
                face = this.dualGraph.Where(v => v.Equals(face)).First() as DualGraph.Face;
            }
            else
            {
                var ret = this.dualGraph.Add(face);
                LogTool.AssertIsTrue(ret);
            }
            this.CheckFaceEdge(face, e1);
            this.CheckFaceEdge(face, e2);
            this.CheckFaceEdge(face, e3);


            return face;
        }

        protected void CheckFaceEdge(DualGraph.Face face, VertexGraph.Edge edge)
        {
            var other = this.dualGraph.FindFaceSharesEdgeWith(face, edge);
            if (other != default)
            {
                this.dualGraph.AddEdge(face, other);
            }
        }

        protected VertexGraph.Vertex AddVert(VertexGraph.Vertex nv)
        {
            if (this.graph.Contains(nv)) return this.graph.Where(v => v.Equals(nv)).First() as VertexGraph.Vertex;
            var ret = this.graph.Add(nv);
            LogTool.AssertIsTrue(ret);
            return nv;
        }

        public float offset = 20;
        protected void OnDrawGizmos()
        {
            using (new GizmosScope(Color.white, this.transform.localToWorldMatrix))
            {
                this.hgraph?.OnDrawGizmos();
            }

            foreach (var (f, other) in this.overlap)
            {
                // var (f, other) = this.overlap.First();
                Gizmos.color = Color.red;
                f.OnDrawGizmos();
                Gizmos.color = Color.green;
                other.OnDrawGizmos();
            }

            var b = 10f;
            foreach (var g in this.noOverlap)
            {
                using (new GizmosScope(Color.magenta, Matrix4x4.Translate(new float3(0, 0, b += offset))))
                {
                    g.OnDrawGizmos();
                }

            }


            using (new GizmosScope(Color.yellow, this.transform.localToWorldMatrix))
            {
                if (this.paths != null && this.paths.Count > 0)
                {
                    // foreach (var p in this.paths)
                    var p = this.paths.First();
                    {
                        foreach (DualGraph.Face f in p)
                        {
                            f.OnDrawGizmos();
                        }

                    }
                }
            }


            // foreach(var (p,n) in ProgressiveMesh.debug)
            // {
            //     Gizmos.DrawSphere(p,0.01f);
            //     Gizmos.DrawLine(p, p + (n * 0.1f));
            // }


            // this.orgmst?.OnDrawGizmos();
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