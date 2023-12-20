//List<Point> Inters = Method.Intersection_Line_Circle(new Point(4,4), new Point(6,6), new Point(5,5), 5);
//Console.WriteLine();





using static G__Interpreter.Figure;

public static class Method
{
    public static List<Point> Intersection_Line_Circle(Point line_p1, Point line_p2, Point circle_center, double radius)
    {
        List<Point> Result = new List<Point>();
        //Si la distancia del centro a la recta es mayor que el radio, no hay intersección
        if (Distancia_Punto_Recta(circle_center, line_p1, line_p2) > radius)
        {
            return Result;
        }
        //Si la distancia del punto a la recta es igual o menor al radio, se intersectan en un solo punto o en dos
        else
        {
            //Si no podemos calcular la pendiente por la via trivial, hay que hacerlo de otra forma
            if (line_p2.X - line_p1.X == 0)
            {
                double X = line_p1.X;
                double Y = circle_center.Y + Math.Sqrt((radius * radius) - ((X - circle_center.X) * (X - circle_center.X)));
                Result.Add(new Point(X, Y));
                Y = circle_center.Y - Math.Sqrt((radius * radius) - ((X - circle_center.X) * (X - circle_center.X)));
                Result.Add(new Point(X, Y));
            }
            else
            {
                //Hallando m y n
                double m = (line_p2.Y - line_p1.Y) / (line_p2.X - line_p1.X);
                double n = line_p2.Y - (m * line_p2.X);
                //Parametrizando
                double A = 1 + (m * m);
                double B = (2 * m * n) - (2 * circle_center.Y * m) - (2 * circle_center.X);
                double C = (circle_center.X * circle_center.X) + (circle_center.Y * circle_center.Y) - (radius * radius) - (2 * n * circle_center.Y) + (n * n);
                double Discriminante = (B * B) - (4 * A * C);
                //Si el dicriminante es 0, tiene una sola intersección
                if (Discriminante == 0)
                {
                    double X = (-B) / (2 * A);
                    double Y = (m * X) + n;
                    Result.Add(new Point(X, Y));
                }
                //Si no es 0, tiene 2 intersecciones
                else
                {
                    double X = ((-B) + Math.Sqrt(Discriminante)) / (2 * A);
                    double Y = (m * X) + n;
                    Result.Add(new Point(X, Y));
                    X = ((-B) - Math.Sqrt(Discriminante)) / (2 * A);
                    Y = (m * X) + n;
                    Result.Add(new Point(X, Y));
                }
            }
        }
        return Result;
    }

    public static List<Point> Intersection_Point_Point(Point p1, Point p2)
    {
        List<Point> intersectionPoints = new List<Point>();

        // Calcula la pendiente entre los dos puntos
        double slope = (p2.Y - p1.Y) / (p2.X - p1.X);

        // Calcula la ordenada al origen 'b' de la recta que pasa por los dos puntos
        double b = p1.Y - (slope * p1.X);

        // Calcula la coordenada x del punto de intersección
        double x = -(b / slope);

        // Calcula la coordenada y del punto de intersección
        double y = slope * x + b;

        // Crea un nuevo punto de intersección con las coordenadas calculadas
        Point intersectionPoint = new Point(x, y);

        intersectionPoints.Add(intersectionPoint);

        return intersectionPoints;
    }

    public static List<Point> Intersection_Point_Line(Point punto, Line recta)
    {
        List<Point> interceptos = new List<Point>();


        // Calcular el valor de 'y' en la recta para el valor 'x' del punto
        double valorY = recta.Slope * punto.X + recta.Intercept;  // Aquí implemente en la clase Recte >>> Intercept

        // Comprobar si el valor de 'y' del punto coincide con el calculado en la recta
        if (Math.Abs(valorY - punto.Y) < 0.001) // Comparación con tolerancia para valores cercanos
        {
            // Agregar el punto de intercepto a la lista
            interceptos.Add(new Point(punto.X, punto.Y));
        }

        return interceptos;
    }
    public static List<Point> Intersections_Point_Segment(Point p, Segment segment)
    {
        List<Point> intersections = new List<Point>();

        double minX = Math.Min(segment.P1.X, segment.P2.X);
        double maxX = Math.Max(segment.P1.X, segment.P2.X);
        double minY = Math.Min(segment.P1.Y, segment.P2.Y);
        double maxY = Math.Max(segment.P1.Y, segment.P2.Y);

        if (p.X >= minX && p.X <= maxX && p.Y >= minY && p.Y <= maxY)
        {
            intersections.Add(p);
        }

        return intersections;
    }

    public static List<Point> Intersections_Pont_Ray(Point point, Ray ray)
    {
        List<Point> interceptos = new List<Point>();

        // Comprobar si la semirecta es vertical (pendiente infinita)
        if (double.IsInfinity(ray.Slope))
        {
            // Comprobar si el punto está en la misma posición x que la semirecta
            if (point.X == ray.P1.X)
            {
                interceptos.Add(new Point(ray.P1.X, ray.P2.Y));
            }
        }
        else
        {
            // Calcular la ecuación de la semirecta: y = mx + b
            double b = ray.P1.Y - ray.Slope * ray.P1.X;

            // Calcular la ecuación de la recta perpendicular al segmento dado por el punto y la semirecta: y = -1/m*x + b'
            double pendientePerpendicular = -1 / ray.Slope;
            double bPerpendicular = point.Y - pendientePerpendicular * point.X;

            // Calcular el punto de intersección entre la semirecta y la rrecta perpendicular
            double xIntercepto = (bPerpendicular - b) / (ray.Slope - pendientePerpendicular);
            double yIntercepto = pendientePerpendicular * xIntercepto + bPerpendicular;

            interceptos.Add(new Point(xIntercepto, yIntercepto));
        }

        return interceptos;
    }

    public static List<Point> Intersections_Point_Circle(Point point, Circle circle)
    {
        List<Point> intersectionPoints = new List<Point>();

        // Distancia entre el punto y el centro de la circunferencia
        double distance = Math.Sqrt(Math.Pow(point.X - circle.Center.X, 2) + Math.Pow(point.Y - circle.Center.Y, 2));

        // Comprobar si el punto está dentro de la circunferencia
        if (distance <= circle.Radius)
        {
            double dX = circle.Center.X - point.X;
            double dY = circle.Center.Y - point.Y;

            // Distancia desde el punto hasta los puntos de intersección
            double h = Math.Sqrt(Math.Pow(circle.Radius, 2) - Math.Pow(distance, 2));

            // Calcular los puntos de intersección
            double intersectionX1 = point.X + (h * dX) / distance;
            double intersectionY1 = point.Y + (h * dY) / distance;
            Point intersectionPoint1 = new Point(intersectionX1, intersectionY1);
            intersectionPoints.Add(intersectionPoint1);

            double intersectionX2 = point.X - (h * dX) / distance;
            double intersectionY2 = point.Y - (h * dY) / distance;
            Point intersectionPoint2 = new Point(intersectionX2, intersectionY2);
            intersectionPoints.Add(intersectionPoint2);
        }

        return intersectionPoints;
    }

    public static List<Point> Intersections_Line_Line(Line line1, Line line2)
    {
        List<Point> puntosInterseccion = new List<Point>();

        double xInterseccion = (line2.Intercept - line1.Intercept) / (line1.Slope - line2.Slope);
        double yInterseccion = (line1.Slope * xInterseccion) + line1.Intercept;

        Point puntoInterseccion = new Point(xInterseccion, yInterseccion);
        puntosInterseccion.Add(puntoInterseccion);

        return puntosInterseccion;
    }

    public static List<Point> Intersections_Line_Segment(Line line, Segment segment)
    {
        List<Point> intersecciones = new List<Point>();

        double x1 = segment.P1.X;
        double y1 = segment.P1.Y;
        double x2 = segment.P2.X;
        double y2 = segment.P2.Y;
        double m = line.Slope;
        double b = line.Intercept;

        // Calcular la pendiente del segmento
        double mSegmento = segment.Slope;

        // Calcular la intersección entre la recta y el segmento
        double xInterseccion = (b - y1 + mSegmento * x1) / (mSegmento - m);
        double yInterseccion = m * xInterseccion + b;

        // Verificar si el punto de intersección está dentro del segmento
        if (xInterseccion >= Math.Min(x1, x2) && xInterseccion <= Math.Max(x1, x2) &&
            yInterseccion >= Math.Min(y1, y2) && yInterseccion <= Math.Max(y1, y2))
        {
            intersecciones.Add(new Point(xInterseccion, yInterseccion));
        }

        return intersecciones;
    }

    public static List<Point> Intersections_Segment_Circle(Segment segment, Circle circle)
    {
        List<Point> puntosIntercepto = new List<Point>();

        // Cálculo de los coeficientes de la ecuación del segmento (y = mx + b)
        double m, b;
        if (segment.P1.X == segment.P2.X)
        {
            // Si el segmento es vertical, no es posible obtener la ecuación
            m = double.PositiveInfinity;
            b = double.NaN;
        }
        else
        {
            // Si el segmento no es vertical, se calcula el coeficiente de pendiente m y el término b
            m = segment.Slope;
            b = segment.Intercept;
        }

        // Cálculo de los puntos de intercepción de la circunferencia y el segmento
        float discriminante = (float)(Math.Pow(2 * m * b - 2 * circle.Center.Y * m - 2 * circle.Center.X, 2) - 4 * (1 + m * m) * (Math.Pow(circle.Center.X, 2) + Math.Pow(b - circle.Center.Y, 2) - Math.Pow(circle.Radius, 2)));

        if (discriminante > 0)
        {
            // Hay dos puntos de intercepción
            double x1 = (float)((-2 * m * b + 2 * circle.Center.Y * m + 2 * circle.Center.X) + Math.Sqrt(discriminante)) / (2 * (1 + m * m));
            double y1 = m * x1 + b;
            puntosIntercepto.Add(new Point(x1, y1));

            double x2 = (float)((-2 * m * b + 2 * circle.Center.Y * m + 2 * circle.Center.X) - Math.Sqrt(discriminante)) / (2 * (1 + m * m));
            double y2 = m * x2 + b;
            puntosIntercepto.Add(new Point(x2, y2));
        }
        else if (discriminante == 0)
        {
            // Hay un punto de intercepción
            double x = (float)((-2 * m * b + 2 * circle.Center.Y * m + 2 * circle.Center.X)) / (2 * (1 + m * m));
            double y = m * x + b;
            puntosIntercepto.Add(new Point(x, y));
        }

        return puntosIntercepto;
    }

    public static List<Point> Intersections_Segment_Segment(Segment seg1, Segment seg2)
    {
        double denominador = (seg1.P1.X - seg1.P2.X) * (seg2.P1.Y - seg2.P2.Y) - (seg1.P1.Y - seg1.P2.Y) * (seg2.P1.X - seg2.P2.X);

        if (denominador == 0) // Los segmentos son paralelos o coincidentes
        {
            return new List<Point>();
        }
        else
        {
            double interseccionX = ((seg1.P1.X * seg1.P2.Y - seg1.P1.Y * seg1.P2.X) * (seg2.P1.X - seg2.P2.X) - (seg1.P1.X - seg1.P2.X) * (seg2.P1.X * seg2.P2.Y - seg2.P1.Y * seg2.P2.X)) / denominador;
            double interseccionY = ((seg1.P1.X * seg1.P2.Y - seg1.P1.Y * seg1.P2.X) * (seg2.P1.Y - seg2.P2.Y) - (seg1.P1.Y - seg1.P2.Y) * (seg2.P1.X * seg2.P2.Y - seg2.P1.Y * seg2.P2.X)) / denominador;

            Point interseccion = new Point(interseccionX, interseccionY);

            if (PuntoEstaEnSegmento(interseccion, seg1) && PuntoEstaEnSegmento(interseccion, seg2))
            {
                return new List<Point>() { interseccion };
            }
            else
            {
                return new List<Point>();
            }
        }
    }

    public static bool PuntoEstaEnSegmento(Point punto, Segment segmento)
    {
        double x = punto.X;
        double y = punto.Y;

        double minX = Math.Min(segmento.P1.X, segmento.P2.X);
        double minY = Math.Min(segmento.P1.Y, segmento.P2.Y);
        double maxX = Math.Max(segmento.P1.X, segmento.P2.X);
        double maxY = Math.Max(segmento.P1.Y, segmento.P2.Y);

        return (x >= minX && x <= maxX && y >= minY && y <= maxY);
    }

    public static List<Point> Intersection_Circle_Circle(Circle circle1, Circle circle2)
    {
        List<Point> intersectionPoints = new List<Point>();

        // Calcula la distancia entre los centros de las circunferencias
        double distance = Math.Sqrt(Math.Pow(circle2.Center.X - circle1.Center.X, 2) + Math.Pow(circle2.Center.Y - circle2.Center.Y, 2));

        // Verifica si las circunferencias no se intersectan
        if (distance > circle1.Radius + circle2.Radius || distance < Math.Abs(circle1.Radius - circle2.Radius))
        {
            // No hay intersección entre las circunferencias
            return intersectionPoints;
        }

        // Calcula la distancia entre los centros de las circunferencias y el punto de intersección
        double a = (Math.Pow(circle1.Radius, 2) - Math.Pow(circle2.Radius, 2) + Math.Pow(distance, 2)) / (2 * distance);
        double h = Math.Sqrt(Math.Pow(circle1.Radius, 2) - Math.Pow(a, 2));

        // Calcula el punto de intersección
        double x = circle1.Center.X + (a * (circle2.Center.X - circle1.Center.X)) / distance;
        double y = circle1.Center.Y + (a * (circle2.Center.Y - circle1.Center.Y)) / distance;

        // Calcula los puntos de intersección ajustando la distancia con el punto de intersección
        double intersectionX1 = x + (h * (circle2.Center.Y - circle1.Center.Y)) / distance;
        double intersectionY1 = y - (h * (circle2.Center.X - circle1.Center.X)) / distance;
        double intersectionX2 = x - (h * (circle2.Center.Y - circle1.Center.Y)) / distance;
        double intersectionY2 = y + (h * (circle2.Center.X - circle1.Center.X)) / distance;

        // Agrega los puntos de intersección a la lista
        intersectionPoints.Add(new Point(intersectionX1, intersectionY1));
        intersectionPoints.Add(new Point(intersectionX2, intersectionY2));

        return intersectionPoints;
    }

    public static List<Point> Intersection_Point_Arc(Point punto, Arc arco)
    {
        List<Point> puntosIntercepcion = new List<Point>();

        // Calcular la distancia desde el punto al centro del arco
        double distancia = Math.Sqrt(Math.Pow(punto.X - arco.Center.X, 2) + Math.Pow(punto.Y - arco.Center.Y, 2));

        // Verificar si el punto está dentro del radio del arco
        if (distancia <= arco.Radius)
        {
            // Calcular el ángulo del punto con respecto al centro del arco
            double angulo = Math.Atan2(punto.Y - arco.Center.Y, punto.X - arco.Center.X) * 180 / Math.PI;

            // Ajustar el ángulo para que esté en el rango de [0, 360]
            if (angulo < 0)
            {
                angulo += 360;
            }

            // Verificar si el ángulo del punto está dentro del rango de ángulos del arco
            if (arco.StartAngle <= arco.EndAngle)
            {
                if (angulo >= arco.StartAngle && angulo <= arco.EndAngle)
                {
                    puntosIntercepcion.Add(punto);
                }
            }
            else
            {
                if (angulo >= arco.StartAngle || angulo <= arco.EndAngle)
                {
                    puntosIntercepcion.Add(punto);
                }
            }
        }

        return puntosIntercepcion;
    }

    public static List<Point> Intersection_Line_Arc(Line line, Arc arco)
    {
        List<Point> intersecciones = new List<Point>();

        double a = 1 + Math.Pow(line.Slope, 2);
        double b = -2 * arco.Center.X + 2 * line.Slope * (line.Intercept - arco.Center.Y);
        double c = Math.Pow(arco.Center.X, 2) + Math.Pow(line.Intercept - arco.Center.Y, 2) - Math.Pow(arco.Radius, 2);

        double discriminante = Math.Pow(b, 2) - 4 * a * c;

        if (discriminante >= 0)
        {
            double x1 = (-b + Math.Sqrt(discriminante)) / (2 * a);
            double y1 = line.Intercept * x1 + line.Intercept;
            intersecciones.Add(new Point(x1, y1));

            if (discriminante > 0)
            {
                double x2 = (-b - Math.Sqrt(discriminante)) / (2 * a);
                double y2 = line.Intercept * x2 + line.Intercept;
                intersecciones.Add(new Point(x2, y2));
            }
        }

        return intersecciones;
    }

    public static List<Point> GetIntersectionPoints(Segment segment, Arc arc)
    {
        List<Point> intersectionPoints = new List<Point>();

        // Calcula la pendiente del segmento
        double segmentSlope = segment.Slope;

        // Calcula la posición relativa de la circunferencia respecto al segmento
        double relativePosition = (segmentSlope * (arc.Center.X - segment.P1.X)) + segment.P1.Y;

        // Comprueba si la circunferencia está por encima o por debajo del segmento
        if (arc.Center.Y > relativePosition + arc.Radius || arc.Center.Y < relativePosition - arc.Radius)
        {
            return intersectionPoints;  // No hay intersección
        }

        // Calcula los puntos de intersección
        double segmentLength = Math.Sqrt(Math.Pow(segment.P2.X - segment.P1.X, 2) + Math.Pow(segment.P2.Y - segment.P1.Y, 2));
        double segmentRotation = Math.Atan2(segment.P2.Y - segment.P1.Y, segment.P2.X - segment.P1.X);
        double startAngle = Math.Atan2(segment.P1.Y - arc.Center.Y, segment.P1.X - arc.Center.X);
        double endAngle = Math.Atan2(segment.P2.Y - arc.Center.Y, segment.P2.X - arc.Center.X);

        // Asegúrate de que los ángulos estén en el rango correcto
        if (startAngle < 0)
        {
            startAngle += 2 * Math.PI;
        }

        if (endAngle < 0)
        {
            endAngle += 2 * Math.PI;
        }


        // Comprueba si el segmento está completamente fuera del arco
        if (endAngle <= arc.StartAngle || startAngle >= arc.EndAngle)
        {
            return intersectionPoints;  // No hay intersección
        }

        // Calcula los puntos de intersección reales
        double t = Math.Acos((Math.Pow(arc.Radius, 2) - Math.Pow(arc.Center.X - segment.P1.X, 2) - Math.Pow(arc.Center.Y - segment.P1.Y, 2)) / (2 * arc.Radius * Math.Sqrt(Math.Pow(arc.Center.X - segment.P1.X, 2) + Math.Pow(arc.Center.Y - segment.P1.Y, 2))));
        double alpha = segmentRotation + startAngle + t;
        double beta = segmentRotation + startAngle - t;

        intersectionPoints.Add(new Point(arc.Center.X + arc.Radius * Math.Cos(alpha), arc.Center.Y + arc.Radius * Math.Sin(alpha)));
        intersectionPoints.Add(new Point(arc.Center.X + arc.Radius * Math.Cos(beta), arc.Center.Y + arc.Radius * Math.Sin(beta)));

        return intersectionPoints;
    }

    public static List<Point> PuntosDeIntercepcion(Arc arco, Circle circle)
    {
        List<Point> puntosIntercepcion = new List<Point>();

        // Obtiene los puntos de intercepción entre la circunferencia y el arco
        double x1, y1, x2, y2;
        double a = Math.Cos(arco.StartAngle) * (arco.Center.X - circle.Center.X) - Math.Sin(arco.StartAngle) * (arco.Center.Y - circle.Center.Y);
        double b = Math.Cos(arco.EndAngle) * (arco.Center.X - circle.Center.X) - Math.Sin(arco.StartAngle) * (arco.Center.Y - circle.Center.Y);
        double c = Math.Sin(arco.StartAngle) * (arco.Center.X - circle.Center.X) + Math.Cos(arco.StartAngle) * (arco.Center.Y - circle.Center.Y);
        double d = Math.Sin(arco.EndAngle) * (arco.Center.X - circle.Center.X) + Math.Cos(arco.EndAngle) * (arco.Center.Y - circle.Center.Y);

        double det = a * d - b * c;

        if (det > 0)
        {
            double sqrtDelta = Math.Sqrt(Math.Pow((a - b), 2) + Math.Pow((c - d), 2));

            x1 = circle.Center.X + (a * d - b * c - sqrtDelta * (d - c)) / (Math.Pow((a - b), 2) + Math.Pow((c - d), 2));
            y1 = circle.Center.Y + (c - d + sqrtDelta * (a - b)) / (Math.Pow((a - b), 2) + Math.Pow((c - d), 2));

            puntosIntercepcion.Add(new Point(x1, y1));

            x2 = circle.Center.X + (a * d - b * c + sqrtDelta * (d - c)) / (Math.Pow((a - b), 2) + Math.Pow((c - d), 2));
            y2 = circle.Center.Y + (c - d - sqrtDelta * (a - b)) / (Math.Pow((a - b), 2) + Math.Pow((c - d), 2));

            puntosIntercepcion.Add(new Point(x2, y2));
        }

        return puntosIntercepcion;
    }

    public static double Distancia_Punto_Recta(Point punto, Point recta_p1, Point recta_p2)
    {
        double distance;
        if (recta_p1.X == recta_p2.X)
        {
            distance = recta_p1.X - punto.X;
        }
        else if (recta_p1.Y == recta_p2.Y)
        {
            distance = recta_p1.Y - punto.Y;
        }
        else
        {
            //Calculando ecuación cartesiana
            double m = (recta_p2.Y - recta_p1.Y) / (recta_p2.X - recta_p1.X);
            double n = recta_p2.Y - (m * recta_p2.X);
            //Declarando parámetros
            double A = m;
            double B = -1;
            double C = n;
            //Calcuando distancia
            distance = ((A * punto.X) + (B * punto.Y) + C) / Math.Sqrt((A * A) + (B * B));
        }
        //Devolver el módulo de la distancia
        if (distance < 0) return -distance;
        return distance;
    }
}
public class Point
{
    public double X;
    public double Y;
    public Point(double x, double y)
    {
        X = x;
        Y = y;
    }
}
