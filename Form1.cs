using OpenCvSharp;
using OpenCvSharp.Aruco;
using OpenCvSharp.Extensions;
using SharpGL;
using SharpGL.Enumerations;
using SharpGL.SceneGraph.Assets;
using System;
using System.Threading;
using System.Windows.Forms;
using Point = OpenCvSharp.Point;

namespace LR5
{
    public partial class Form1 : Form
    {
        OpenGL gl;
        Thread cameraThread;
        bool runVideo;
        VideoCapture capture;
        DetectorParameters detectorParam = new DetectorParameters();
        Mat matInput;
        Point2f[][] corners;
        int[] ids = new int[] { };
        Mat rvec, tvec;
        bool[] displayMode = new bool[] { true, false, false };
        bool showOneMarker;
        int idMark = 0;
        readonly Mat cameraMatrix = new Mat(3, 3, MatType.CV_64F, new double[] { 968.08624052, 0, 644.63057786, 0, 955.40821957, 364.58219136, 0, 0, 1 });
        readonly Mat distCoeffs = new Mat(5, 1, MatType.CV_64F, new double[] { -4.73423959e-02, -1.25875642e+00, 1.12354237e-04, 4.72701099e-03, 6.96144663e+00 });
        readonly Point3f[] objectPoints = new Point3f[] { new Point3f(1.0f, 1.0f, 0), new Point3f(-1.0f, 1.0f, 0), new Point3f(-1.0f, -1.0f, 0), new Point3f(1.0f, -1.0f, 0) };
        readonly Scalar[] colorsScalar = new Scalar[] { Scalar.Red, Scalar.Green, Scalar.Black, Scalar.Blue };
        readonly Dictionary dictionary;
        readonly Texture cameraTexture = new Texture();

        private void button1_Click(object sender, EventArgs e)
        {
            if (runVideo)
            {
                runVideo = false;
                timer1.Stop();
                DisposeVideo();
                button1.Text = "Старт";
            }
            else
            {
                timer1.Start();
                runVideo = true;
                matInput = new Mat();
                capture = new VideoCapture(0)
                {
                    FrameHeight = 720,
                    FrameWidth = 1280,
                    AutoFocus = true
                };

                cameraThread = new Thread(new ThreadStart(CaptureCameraCallback));
                cameraThread.Start();
                button1.Text = "Стоп";
            }

        }
        private void DisposeVideo()
        {
            if (cameraThread != null && cameraThread.IsAlive) cameraThread.Abort();
            matInput?.Dispose();
            capture?.Dispose();
        }
        public void PrintCube(float size = 1.0f)
        {
            gl.LineWidth(2);
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 0.5, 0.5);
            gl.Vertex(0.0f, 0, 0);
            gl.Vertex(size * 1.5, 0, 0);

            gl.Color(0.5, 1.0f, 0.5);
            gl.Vertex(0.0f, 0, 0);
            gl.Vertex(0, size * 1.5, 0);

            gl.Color(0.5, 0.5, 1.0f);
            gl.Vertex(0, 0, 0.0f);
            gl.Vertex(0, 0, size * 1.5);
            gl.End();

            gl.Color(0.5, 0.5, 0.5f);
            gl.PolygonMode(OpenGL.GL_FRONT_AND_BACK, OpenGL.GL_LINE);
            gl.Begin(BeginMode.Quads);

            gl.Vertex(-size, size, -size);  // top
            gl.Vertex(size, size, -size);
            gl.Vertex(size, size, size);
            gl.Vertex(-size, size, size);

            gl.Vertex(-size, -size, -size);  // bott0m
            gl.Vertex(size, -size, -size);
            gl.Vertex(size, -size, size);
            gl.Vertex(-size, -size, size);

            gl.Vertex(-size, -size, -size);  // left
            gl.Vertex(-size, size, -size);
            gl.Vertex(-size, size, size);
            gl.Vertex(-size, -size, size);

            gl.Vertex(size, -size, -size);  // right
            gl.Vertex(size, size, -size);
            gl.Vertex(size, size, size);
            gl.Vertex(size, -size, size);
            gl.End();
        }
        public Form1()
        {
            InitializeComponent();
            dictionary = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.Dict6X6_50);

        }
        private void openGLControl1_OpenGLInitialized(object sender, EventArgs e)
        {
            gl = openGLControl1.OpenGL;
            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
            gl.Enable(OpenGL.GL_TEXTURE_2D);
        }
        private void openGLControl1_OpenGLDraw(object sender, RenderEventArgs args)
        {
            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
            gl.MatrixMode(OpenGL.GL_PROJECTION);
            gl.LoadIdentity();
            gl.Perspective(45, openGLControl1.Width / (double)openGLControl1.Height, 0.1f, 100.0f);
            gl.MatrixMode(OpenGL.GL_MODELVIEW);
            gl.LoadIdentity();

            if (runVideo && !matInput.Empty())
            {
                if (ids.Length > 0)
                {
                    for (int i = 0; i < ids.Length; i++)
                    {
                        if (showOneMarker && ids[i] != idMark) continue;
                        rvec = new Mat();
                        tvec = new Mat();
                        Cv2.SolvePnP(InputArray.Create(objectPoints), InputArray.Create(corners[i]), cameraMatrix, distCoeffs, rvec, tvec);
                        if (!rvec.Empty() && !tvec.Empty())
                        {
                            gl.LoadMatrix(TransitionToMark(rvec, tvec));
                            if (displayMode[0]) // Отладочный режим
                            {
                                gl.Translate(0.0f, 0.0f, -1.0f);
                                PrintCube();
                            }
                            else    // Стандартый режим
                            {
                                PrintCube();
                            }
                        }
                    }
                }
                if (!displayMode[1]) ShowCameraGL();
            }
            if (displayMode[1]) // Визуализация сцены
            {
                gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
                gl.LoadIdentity();
                gl.Translate(0, 0.0f, -5.0f);
                gl.Color(0.1f, 0.0f, 1.0f);
                gl.PolygonMode(OpenGL.GL_FRONT_AND_BACK, OpenGL.GL_FILL);
                gl.Begin(BeginMode.Quads);
                gl.Vertex(-2.0f, -1.5f, -2.0f); // b0ttom 
                gl.Vertex(2.0f, -1.5f, -2.0f);
                gl.Vertex(2.0f, -1.5f, 3.0f);
                gl.Vertex(-2.0f, -1.5f, 3.0f);

                gl.Color(1.0f, 0.0f, 0.0f);
                gl.Vertex(-2.5f, -2, -4f);  // left
                gl.Vertex(-2.5f, 2, -4f);
                gl.Vertex(-2.5f, 2, 2.5f);
                gl.Vertex(-2.5f, -2, 2.5f);

                gl.Color(0.0f, 1.0f, 0.0f);
                gl.Vertex(2.5f, -2, -4f);  // right
                gl.Vertex(2.5f, 2, -4f);
                gl.Vertex(2.5f, 2, 2.5f);
                gl.Vertex(2.5f, -2, 2.5f);

                gl.Color(0.5f, 1.0f, 0.5f);
                gl.Vertex(-2.0f, 1.6f, -2.0f);  // back
                gl.Vertex(2.0f, 1.6f, -2.0f);
                gl.Vertex(2.0f, -2f, -2.0f);
                gl.Vertex(-2.0f, -2f, -2.0f);

                gl.End();

                gl.Translate(0.1, 0.1, -0.5f);
                gl.Rotate(-30, 15, 0);
                

                PrintCube(); // Отображение данных

                Texture markTexture = new Texture();
                markTexture.Create(gl, "mark.bmp");
                markTexture.Bind(gl);
                gl.PolygonMode(OpenGL.GL_FRONT_AND_BACK, OpenGL.GL_FILL);
                gl.Color(1.0f, 1.0f, 1.0f);
                gl.Begin(OpenGL.GL_QUADS);
                gl.TexCoord(0.0f, 0.0f); gl.Vertex(1.0f, 1.0f, 0);
                gl.TexCoord(1.0f, 0.0f); gl.Vertex(-1.0f, 1.0f, 0);
                gl.TexCoord(1.0f, 1.0f); gl.Vertex(-1.0f, -1.0f, 0);
                gl.TexCoord(0.0f, 1.0f); gl.Vertex(1.0f, -1.0f, 0);

                gl.End();
                markTexture.Destroy(gl);

            }
            gl.Flush();
        }
        private double[] TransitionToMark(Mat localRvec, Mat localTvec)
        {

            gl.LoadIdentity();
            Mat rotation = new Mat();
            Mat viewMatrix = Mat.Zeros(rows: 4, 4, MatType.CV_64F);
            Cv2.Rodrigues(localRvec, rotation);
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    viewMatrix.At<double>(row, col) = rotation.At<double>(row, col);
                }
                viewMatrix.At<double>(row, 3) = localTvec.At<double>(row, 0);
            }
            viewMatrix.At<double>(3, 3) = 1.0f;

            Mat cvToGl = Mat.Zeros(rows: 4, 4, MatType.CV_64F);
            cvToGl.At<double>(0, 0) = -1.0f; // Invert the x axis из-за отзеркаливания
            cvToGl.At<double>(1, 1) = -1.0f; // Invert the y axis
            cvToGl.At<double>(2, 2) = -1.0f; // invert the z axis
            cvToGl.At<double>(3, 3) = 1.0f;
            viewMatrix = cvToGl * viewMatrix;

            Mat glViewMatrix = Mat.Zeros(rows: 4, 4, MatType.CV_64F);
            Cv2.Transpose(viewMatrix, glViewMatrix);
            double[] doubleArray = new double[glViewMatrix.Rows * glViewMatrix.Cols];

            for (int row = 0; row < glViewMatrix.Rows; row++)
            {
                for (int col = 0; col < glViewMatrix.Cols; col++)
                {
                    doubleArray[row * glViewMatrix.Cols + col] = glViewMatrix.At<double>(row, col);
                }
            }
            return doubleArray;
        }

        private void ShowCameraGL()
        {
            gl.MatrixMode(OpenGL.GL_PROJECTION);
            gl.LoadIdentity();
            gl.Ortho(-1, 1, -1, 1, 0.1f, 100);
            gl.MatrixMode(OpenGL.GL_MODELVIEW);
            gl.LoadIdentity();
            gl.Translate(0.0f, 0.0f, -100.0f);

            cameraTexture.Create(gl, matInput.ToBitmap());
            cameraTexture.Bind(gl);
            gl.PolygonMode(OpenGL.GL_FRONT_AND_BACK, OpenGL.GL_FILL);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Begin(OpenGL.GL_QUADS);
            gl.TexCoord(0.0f, 0.0f); gl.Vertex(1.0f, 1.0f, 0);
            gl.TexCoord(1.0f, 0.0f); gl.Vertex(-1.0f, 1.0f, 0);
            gl.TexCoord(1.0f, 1.0f); gl.Vertex(-1.0f, -1.0f, 0);
            gl.TexCoord(0.0f, 1.0f); gl.Vertex(1.0f, -1.0f, 0);

            gl.End();
            cameraTexture.Destroy(gl);
        }

        private void CaptureCameraCallback()
        {
            while (runVideo)
            {
                if (displayMode[1]) continue;
                matInput = capture.RetrieveMat();
                CvAruco.DetectMarkers(matInput, dictionary, out corners, out ids, detectorParam, out _);
                if (ids.Length > 0)
                {
                    if (displayMode[0])
                    {
                        for (int j = 0; j < ids.Length; j++)
                        {
                            if (showOneMarker && ids[j] != idMark) continue;

                            Point center = new Point((corners[j][0].X + corners[j][1].X + corners[j][2].X + corners[j][3].X) / 4,
                                                     (corners[j][0].Y + corners[j][1].Y + corners[j][2].Y + corners[j][3].Y) / 4);

                            Cv2.Circle(matInput, center, 10, Scalar.Yellow, 2);
                            Cv2.PutText(matInput, ids[j].ToString(), center, HersheyFonts.HersheySimplex, 1.5, Scalar.Black, 2);

                            for (int i = 0; i < 4; i++)
                            {
                                Cv2.Line(matInput, corners[j][i].ToPoint(), corners[j][(i + 1) % 4].ToPoint(), Scalar.Red, 3);
                                Cv2.Circle(matInput, corners[j][i].ToPoint(), 5, colorsScalar[i], 2);
                            }
                        }
                    }

                }
                Invoke(new Action(() =>
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                }));
            }
        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            displayMode[0] = true;
            displayMode[1] = false;
            displayMode[2] = false;
        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            displayMode[0] = false;
            displayMode[1] = true;
            displayMode[2] = false;
        }

        private void radioButton3_CheckedChanged(object sender, EventArgs e)
        {
            displayMode[0] = false;
            displayMode[1] = false;
            displayMode[2] = true;
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            showOneMarker = checkBox1.Checked;
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            int.TryParse(textBox1.Text, out idMark);
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            dataGridView1.Rows.Clear();
            dataGridView1.Columns.Add("IdMark", "IdMark");
            dataGridView1.Columns.Add("Coords", "Coords");
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (ids.Length > 0)
            {
                dataGridView1.Rows.Clear();

                for (int i = 0; i < ids.Length; i++)
                {
                    Point center = new Point((corners[i][0].X + corners[i][1].X + corners[i][2].X + corners[i][3].X) / 4,
                                             (corners[i][0].Y + corners[i][1].Y + corners[i][2].Y + corners[i][3].Y) / 4);

                    dataGridView1.Rows.Add($"{ids[i]}", $"{center}");

                }
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            DisposeVideo();
        }
    }
}