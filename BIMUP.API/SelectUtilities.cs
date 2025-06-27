using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BIMUP.API
{
    public class SelectUtilities
    {
        /// <summary>
        /// Chọn nhiều đối tượng theo loại và prompt chỉ định
        /// </summary>
        /// <param name="typeName">Tên lớp đối tượng cần chọn, ví dụ: "Polyline", "AECC_TIN_SURFACE"</param>
        /// <param name="prompt">Chuỗi hiển thị yêu cầu chọn</param>
        /// <returns>Danh sách ObjectId của các đối tượng phù hợp</returns>
        public static ObjectIdCollection SelectObjects(string typeName, string prompt)
        {
            ObjectIdCollection selectedIds = new ObjectIdCollection();

            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;

            PromptSelectionOptions options = new PromptSelectionOptions()
            {
                MessageForAdding = prompt,
                AllowDuplicates = false,
            };

            SelectionFilter filter = new SelectionFilter(new TypedValue[]
            {
                new TypedValue((int)DxfCode.Start, typeName)
            });

            PromptSelectionResult result = ed.GetSelection(options, filter);

            if (result.Status == PromptStatus.OK)
            {
                SelectionSet selectionSet = result.Value;
                foreach (SelectedObject selectedObject in selectionSet)
                {
                    if (selectedObject != null)
                    {
                        selectedIds.Add(selectedObject.ObjectId);
                    }
                }
            }
            return selectedIds;
        }

        public static ObjectIdCollection SelectObjectsWithLayer(string typeName, string prompt, string layername)
        {
            ObjectIdCollection selectedIds = new ObjectIdCollection();

            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;

            PromptSelectionOptions options = new PromptSelectionOptions()
            {
                MessageForAdding = prompt,
                AllowDuplicates = false,
            };

            SelectionFilter filter = new SelectionFilter(new TypedValue[]
            {
                new TypedValue((int)DxfCode.Start, typeName),
                new TypedValue((int)DxfCode.LayerName, layername)
            });

            PromptSelectionResult result = ed.GetSelection(options, filter);

            if (result.Status == PromptStatus.OK)
            {
                SelectionSet selectionSet = result.Value;
                foreach (SelectedObject selectedObject in selectionSet)
                {
                    if (selectedObject != null)
                    {
                        selectedIds.Add(selectedObject.ObjectId);
                    }
                }
            }
            return selectedIds;
        }
    }
}
