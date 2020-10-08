using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using UnityTools.Attributes;
using UnityTools.Debuging.EditorTool;

namespace UnityPaperModel
{
    public class ObjReader : MonoBehaviour
    {
        [SerializeField, FileNamePopup("*.obj")] protected string fileName;
        protected Dictionary<int, VertexGraph> graphs = new Dictionary<int, VertexGraph>();
        protected void Start()
        {
            var path = Path.Combine(Application.streamingAssetsPath, this.fileName);
            var data = File.ReadAllLines(path);


            var map = new Dictionary<int, UnityTools.Common.IVertex>();
            var count = 1;
            foreach (var line in data)
            {
                if (line[0] == 'v')
                {
                    var v = new VertexGraph.Vertex();
                    var pos = line.Split(' ');
                    v.Position = new float3(float.Parse(pos[1]), float.Parse(pos[2]), float.Parse(pos[3]));
                    map.Add(count++, v);
                }
                else
                if (line[0] == 'e')
                {
                    var id = line.Split(' ');
                    var e1 = int.Parse(id[1]);
                    var e2 = int.Parse(id[2]);
                    var group = int.Parse(id[3]);
                    if (this.graphs.ContainsKey(group) == false)
                    {
                        this.graphs.Add(group, new VertexGraph());
                    }

                    this.graphs[group].AddVertex(map[e1]);
                    this.graphs[group].AddVertex(map[e2]);
                    this.graphs[group].AddEdge(map[e1], map[e2]);
                }

            }
        }


        private Dictionary<int, Color> colorMap = new Dictionary<int, Color>()
        {
            {0, Color.red},
            {1, Color.cyan},
            {2, Color.green},
            {3, Color.blue}
        };

        protected void OnDrawGizmos()
        {
            var offset = 0;
            foreach (var g in this.graphs)
            {

                using (new GizmosScope(this.colorMap[g.Key], Matrix4x4.Translate(new float3(offset, 0, 0))))
                {
                    g.Value.OnDrawGizmos();
                }

                offset += 50;
            }
        }
    }
}