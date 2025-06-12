using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Runtime;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data.Common;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;
using Application = Autodesk.AutoCAD.ApplicationServices.Application;

namespace BIMUP.Tools
{
    public static class Test
    {
        [CommandMethod("EXPORTSELECTEDTEXT")]
        public static void ExportSelectedText()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            // Yêu cầu người dùng chọn đối tượng
            PromptSelectionResult psr = ed.GetSelection();

            if (psr.Status != PromptStatus.OK)
            {
                ed.WriteMessage("\nThao tác chọn đã bị hủy. Không có văn bản nào được xuất.");
                return;
            }

            SelectionSet ss = psr.Value;

            if (ss.Count == 0)
            {
                ed.WriteMessage("\nKhông có đối tượng nào được chọn. Không có văn bản nào được xuất.");
                return;
            }

            // Mở hộp thoại lưu file
            SaveFileDialog saveFileDialog = new SaveFileDialog();
            saveFileDialog.Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*";
            saveFileDialog.Title = "Chọn nơi lưu file Text từ các đối tượng đã chọn";
            saveFileDialog.FileName = "ExportedSelectedText.txt"; // Tên file mặc định

            if (saveFileDialog.ShowDialog() != DialogResult.OK)
            {
                ed.WriteMessage("\nThao tác đã bị hủy bởi người dùng.");
                return;
            }

            string filePath = saveFileDialog.FileName;
            StringBuilder sb = new StringBuilder();

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                foreach (SelectedObject acObj in ss)
                {
                    if (acObj == null || acObj.ObjectId.IsNull)
                        continue;

                    Entity ent = tr.GetObject(acObj.ObjectId, OpenMode.ForRead) as Entity;

                    if (ent is DBText)
                    {
                        DBText text = ent as DBText;
                        sb.AppendLine(text.TextString);
                    }
                    else if (ent is MText)
                    {
                        MText mtext = ent as MText;
                        sb.AppendLine(mtext.Contents);
                    }
                    else if (ent is AttributeReference)
                    {
                        AttributeReference attRef = ent as AttributeReference;
                        sb.AppendLine(attRef.TextString);
                    }
                    else if (ent is BlockReference)
                    {
                        BlockReference br = ent as BlockReference;
                        Autodesk.AutoCAD.DatabaseServices.AttributeCollection attCol = br.AttributeCollection;
                        foreach (ObjectId attId in attCol)
                        {
                            AttributeReference attRef = tr.GetObject(attId, OpenMode.ForRead) as AttributeReference;
                            if (attRef != null)
                            {
                                sb.AppendLine(attRef.TextString);
                            }
                        }
                    }
                    else if (ent is AttributeDefinition)
                    {
                        AttributeDefinition attDef = ent as AttributeDefinition;
                        sb.AppendLine(attDef.TextString);
                    }
                }
                tr.Commit();
            }

            try
            {
                File.WriteAllText(filePath, sb.ToString(), Encoding.UTF8);
                ed.WriteMessage($"\nĐã xuất văn bản từ các đối tượng đã chọn thành công ra: {filePath}");
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi khi xuất văn bản từ các đối tượng đã chọn: {ex.Message}");
            }
        }


        [CommandMethod("COPYTEXTTOCLIPBOARD")]
        public static void CopyToClipboard()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            // Yêu cầu người dùng chọn đối tượng
            PromptSelectionResult psr = ed.GetSelection();

            if (psr.Status != PromptStatus.OK)
            {
                ed.WriteMessage("\nThao tác chọn đã bị hủy. Không có văn bản nào được sao chép.");
                return;
            }

            SelectionSet ss = psr.Value;

            if (ss.Count == 0)
            {
                ed.WriteMessage("\nKhông có đối tượng nào được chọn. Không có văn bản nào được sao chép.");
                return;
            }

            StringBuilder sb = new StringBuilder();

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                foreach (SelectedObject acObj in ss)
                {
                    if (acObj == null || acObj.ObjectId.IsNull)
                        continue;

                    Entity ent = tr.GetObject(acObj.ObjectId, OpenMode.ForRead) as Entity;

                    if (ent is DBText)
                    {
                        DBText text = ent as DBText;
                        sb.AppendLine(text.TextString);
                    }
                    else if (ent is MText)
                    {
                        MText mtext = ent as MText;
                        sb.AppendLine(mtext.Contents);
                    }
                    else if (ent is AttributeReference)
                    {
                        AttributeReference attRef = ent as AttributeReference;
                        sb.AppendLine(attRef.TextString);
                    }
                    else if (ent is BlockReference)
                    {
                        BlockReference br = ent as BlockReference;
                        Autodesk.AutoCAD.DatabaseServices.AttributeCollection attCol = br.AttributeCollection;
                        foreach (ObjectId attId in attCol)
                        {
                            AttributeReference attRef = tr.GetObject(attId, OpenMode.ForRead) as AttributeReference;
                            if (attRef != null)
                            {
                                sb.AppendLine(attRef.TextString);
                            }
                        }
                    }
                    else if (ent is AttributeDefinition)
                    {
                        AttributeDefinition attDef = ent as AttributeDefinition;
                        sb.AppendLine(attDef.TextString);
                    }
                }
                tr.Commit();
            }

            try
            {
                // Copy the collected text to the clipboard
                if (sb.Length > 0)
                {
                    // The Clipboard class requires a STA thread. 
                    // Since AutoCAD commands run on an MTA thread, we need to use a separate thread or invoke.
                    // For simplicity in a direct AutoCAD command, we'll try direct access,
                    // but for robust applications, consider Application.Invoke or a dedicated STA thread.

                    // A more robust way to handle STA apartment requirement for Clipboard:
                    System.Threading.Thread staThread = new System.Threading.Thread(() => Clipboard.SetText(sb.ToString()));
                    staThread.SetApartmentState(System.Threading.ApartmentState.STA);
                    staThread.Start();
                    staThread.Join(); // Wait for the thread to complete

                    ed.WriteMessage("\nĐã sao chép văn bản từ các đối tượng đã chọn vào Clipboard thành công.");
                }
                else
                {
                    ed.WriteMessage("\nKhông tìm thấy văn bản nào trong các đối tượng đã chọn để sao chép.");
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi khi sao chép văn bản vào Clipboard: {ex.Message}");
            }
        }
    }
}
