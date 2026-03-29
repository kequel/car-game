/****************************************************
    Virtual Collaborative Teams
    Klient - architektura KLIENT-SERWER (unicast)
    Zmiany wzgledem wersji multicast:
      - multi_reciv / multi_send -> uni_recv / uni_send
      - wysylanie tylko do serwera (SERVER_IP, port 1001)
      - odbior tylko od serwera (port 1002)
      - kolizje wykrywa serwer, klient tylko reaguje
 ****************************************************/

#include <windows.h>
#include <math.h>
#include <time.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include <iterator>
#include <map>

#include "objects.h"
#include "graphics.h"
#include "net.h"
using namespace std;

#define FRAME_STATE     1
#define FRAME_COLLISION 2
#define FRAME_REGISTER  3
#define FRAME_INACTIVE  4

// zmien na IP serwera (ipconfig), lub 127.0.0.1 jesli serwer na tym samym komputerze
#define SERVER_IP        "127.0.0.1"
#define SERVER_PORT      htons(1001)   // serwer odbiera na 1001
#define CLIENT_RECV_PORT htons(1002)   // klient odbiera na 1002

FILE* f = fopen("vct_log.txt", "w");

MovableObject* my_car;
Environment env;
map<int, MovableObject*> other_cars;

float avg_cycle_time = 1.0f / 60.0f; // default dt (assume 60 FPS) to allow simulation immediately
long time_of_cycle = 0, number_of_cyc = 0;
long time_start = clock();

unicast_net* uni_recv = NULL;   // odbior od serwera (port 1002)
unicast_net* uni_send = NULL;   // wysylanie do serwera (losowy port lokalny)

HANDLE threadReciv;
HWND main_window;
CRITICAL_SECTION m_cs;

bool if_SHIFT_pressed = false;
bool if_ID_visible = true;
bool if_mouse_control = false;
int  mouse_cursor_x = 0, mouse_cursor_y = 0;

extern ViewParams viewpar;
long duration_of_day = 800;

struct Frame
{
    int  iID;
    int  type;
    ObjectState state;
    long sending_time;
    int  iID_receiver;
};

DWORD WINAPI ReceiveThreadFun(void* ptr)
{
    unicast_net* pnet = (unicast_net*)ptr;
    Frame frame;
    unsigned long senderIP = 0;

    while (1)
    {
        int frame_size = pnet->reciv((char*)&frame, &senderIP, sizeof(Frame));
        if (frame_size <= 0) continue;
        printf("odebrano ramke: iID=%d type=%d my_iID=%d\n", frame.iID, frame.type, my_car->iID);

        EnterCriticalSection(&m_cs);

        if (frame.type == FRAME_INACTIVE)
        {
            if (other_cars.count(frame.iID))
            {
                delete other_cars[frame.iID];
                other_cars.erase(frame.iID);
            }
            LeaveCriticalSection(&m_cs);
            continue;
        }

        if (frame.type == FRAME_COLLISION && frame.iID_receiver == my_car->iID)
        {
            my_car->is_collided = true;
            my_car->collision_point = frame.state.vPos;
            LeaveCriticalSection(&m_cs);
            continue;
        }

        if (frame.iID != my_car->iID)
        {
            if (other_cars.find(frame.iID) == other_cars.end() || other_cars[frame.iID] == NULL)
            {
                MovableObject* ob = new MovableObject();
                ob->iID = frame.iID;
                other_cars[frame.iID] = ob;
            }
            other_cars[frame.iID]->ChangeState(frame.state);

            if (my_car->is_collided)
            {
                float dist = (my_car->state.vPos - frame.state.vPos).length();
                if (dist >= (my_car->bounding_radius + other_cars[frame.iID]->bounding_radius))
                    my_car->is_collided = false;
            }
        }

        LeaveCriticalSection(&m_cs);
    }
    return 1;
}

void InteractionInitialisation()
{
    DWORD dwThreadId;

    my_car = new MovableObject();
    my_car->iID = 100 + (int)((time(NULL) + GetCurrentProcessId()) % 900);
    time_of_cycle = clock();

    // Klient odbiera na porcie 1002
    uni_recv = new unicast_net(CLIENT_RECV_PORT);

    // Klient wysyla z losowego portu, ale cel to SERVER_IP:1001
    uni_send = new unicast_net(htons(0));
    uni_send->udpServAddr.sin_port = SERVER_PORT;  // ustaw port docelowy serwera

    // Wyślij ramkę rejestracyjną natychmiast, żeby serwer od razu zna o naszym kliencie
    {
        Frame reg;
        reg.state = my_car->State();
        reg.iID = my_car->iID;
        reg.type = FRAME_REGISTER;
        reg.sending_time = clock();
        reg.iID_receiver = 0;
        uni_send->send((char*)&reg, SERVER_IP, sizeof(Frame));
    }

    threadReciv = CreateThread(
        NULL, 0,
        ReceiveThreadFun,
        (void*)uni_recv,
        NULL,
        &dwThreadId);
    SetThreadPriority(threadReciv, THREAD_PRIORITY_HIGHEST);

    printf("Klient uruchomiony. Serwer: %s\n", SERVER_IP);
}

void VirtualWorldCycle()
{
    number_of_cyc++;

    if (number_of_cyc % 50 == 0)
    {
        char text[256];
        long prev_time = time_of_cycle;
        time_of_cycle = clock();
        float fFps = (50 * CLOCKS_PER_SEC) / (float)(time_of_cycle - prev_time);
        if (fFps != 0) avg_cycle_time = 1.0f / fFps; else avg_cycle_time = 1;

        sprintf(text,
            "WZR-lab 2025/26 (lato) temat 2 klient-serwer (%0.0f fps  %0.2fms)",
            fFps, 1000.0f / fFps);
        SetWindowText(main_window, text);
    }

    my_car->Simulation(avg_cycle_time);

    Frame frame;
    frame.state = my_car->State();
    frame.iID = my_car->iID;
    frame.type = FRAME_STATE;
    frame.sending_time = clock();
    frame.iID_receiver = 0;

    uni_send->send((char*)&frame, SERVER_IP, sizeof(Frame));
}

void EndOfInteraction()
{
    fprintf(f, "Koniec interakcji\n");
    fclose(f);
}

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
HDC g_context = NULL;

int WINAPI WinMain(HINSTANCE hInstance,
    HINSTANCE hPrevInstance,
    LPSTR     lpCmdLine,
    int       nCmdShow)
{
    InitializeCriticalSection(&m_cs);

    MSG message;
    WNDCLASS main_class;
    static char class_name[] = "Klasa_KlientSerwer";

    main_class.style = CS_HREDRAW | CS_VREDRAW;
    main_class.lpfnWndProc = WndProc;
    main_class.cbClsExtra = 0;
    main_class.cbWndExtra = 0;
    main_class.hInstance = hInstance;
    main_class.hIcon = 0;
    main_class.hCursor = LoadCursor(0, IDC_ARROW);
    main_class.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);
    main_class.lpszMenuName = "Menu";
    main_class.lpszClassName = class_name;

    RegisterClass(&main_class);

    main_window = CreateWindow(class_name,
        "WZR-lab 2025/26 (lato) temat 2 - klient-serwer",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,
        100, 50, 950, 650, NULL, NULL, hInstance, NULL);

    ShowWindow(main_window, nCmdShow);
    UpdateWindow(main_window);

    ZeroMemory(&message, sizeof(message));
    while (message.message != WM_QUIT)
    {
        if (PeekMessage(&message, NULL, 0U, 0U, PM_REMOVE))
        {
            TranslateMessage(&message);
            DispatchMessage(&message);
        }
        else
        {
            VirtualWorldCycle();
            InvalidateRect(main_window, NULL, FALSE);
        }
    }
    return (int)message.wParam;
}

LRESULT CALLBACK WndProc(HWND main_window, UINT message_code, WPARAM wParam, LPARAM lParam)
{
    switch (message_code)
    {
    case WM_CREATE:
    {
        g_context = GetDC(main_window);
        srand((unsigned)time(NULL));
        GraphicsInitialisation(g_context);
        InteractionInitialisation();
        SetTimer(main_window, 1, 10, NULL);
        return 0;
    }
    case WM_PAINT:
    {
        PAINTSTRUCT paint;
        HDC context = BeginPaint(main_window, &paint);
        DrawScene();
        SwapBuffers(context);
        EndPaint(main_window, &paint);
        return 0;
    }
    case WM_TIMER:
        return 0;
    case WM_SIZE:
    {
        int cx = LOWORD(lParam), cy = HIWORD(lParam);
        WindowResize(cx, cy);
        return 0;
    }
    case WM_DESTROY:
    {
        EndOfInteraction();
        EndOfGraphics();
        ReleaseDC(main_window, g_context);
        KillTimer(main_window, 1);
        DWORD ExitCode;
        GetExitCodeThread(threadReciv, &ExitCode);
        TerminateThread(threadReciv, ExitCode);
        other_cars.clear();
        PostQuitMessage(0);
        return 0;
    }
    case WM_LBUTTONDOWN:
        if (if_mouse_control) my_car->F = 30.0f;
        break;
    case WM_RBUTTONDOWN:
        if (if_mouse_control) my_car->F = -5.0f;
        break;
    case WM_MBUTTONDOWN:
        if_mouse_control = 1 - if_mouse_control;
        if (if_mouse_control) my_car->if_keep_steer_wheel = true;
        else my_car->if_keep_steer_wheel = false;
        mouse_cursor_x = LOWORD(lParam);
        mouse_cursor_y = HIWORD(lParam);
        break;
    case WM_LBUTTONUP:
        if (if_mouse_control) my_car->F = 0.0f;
        break;
    case WM_RBUTTONUP:
        if (if_mouse_control) my_car->F = 0.0f;
        break;
    case WM_MOUSEMOVE:
        if (if_mouse_control)
        {
            int x = LOWORD(lParam), y = HIWORD(lParam);
            float wheel_angle = (float)(mouse_cursor_x - x) / 20.0f;
            if (wheel_angle > 60) wheel_angle = 60;
            if (wheel_angle < -60) wheel_angle = -60;
            my_car->state.steering_angle = (float)PI * wheel_angle / 180.0f;
        }
        break;
    case WM_KEYDOWN:
        switch (LOWORD(wParam))
        {
        case VK_SHIFT:  if_SHIFT_pressed = true; break;
        case VK_SPACE:  my_car->breaking_factor = 1.0f; break;
        case VK_UP:     my_car->F = 100.0f; break;
        case VK_DOWN:   my_car->F = -70.0f;  break;
        case VK_LEFT:
            if (my_car->steer_wheel_speed < 0) { my_car->steer_wheel_speed = 0; my_car->if_keep_steer_wheel = true; }
            else my_car->steer_wheel_speed = if_SHIFT_pressed ? 0.5f : 0.25f / 8.0f;
            break;
        case VK_RIGHT:
            if (my_car->steer_wheel_speed > 0) { my_car->steer_wheel_speed = 0; my_car->if_keep_steer_wheel = true; }
            else my_car->steer_wheel_speed = if_SHIFT_pressed ? -0.5f : -0.25f / 8.0f;
            break;
        case 'I': if_ID_visible = 1 - if_ID_visible; break;
        case 'W': if (viewpar.cam_distance > 0.5f) viewpar.cam_distance /= 1.2f; else viewpar.cam_distance = 0; break;
        case 'S': if (viewpar.cam_distance > 0) viewpar.cam_distance *= 1.2f; else viewpar.cam_distance = 0.5f; break;
        case 'Q':
            if (!viewpar.tracking) {
                viewpar.top_view = 1 - viewpar.top_view;
                if (viewpar.top_view) {
                    viewpar.cam_pos_1 = viewpar.cam_pos; viewpar.cam_direct_1 = viewpar.cam_direct; viewpar.cam_vertical_1 = viewpar.cam_vertical;
                    viewpar.cam_distance_1 = viewpar.cam_distance; viewpar.cam_angle_1 = viewpar.cam_angle;
                    viewpar.cam_pos = viewpar.cam_pos_2; viewpar.cam_direct = viewpar.cam_direct_2; viewpar.cam_vertical = viewpar.cam_vertical_2;
                    viewpar.cam_distance = viewpar.cam_distance_2; viewpar.cam_angle = viewpar.cam_angle_2;
                }
                else {
                    viewpar.cam_pos_2 = viewpar.cam_pos; viewpar.cam_direct_2 = viewpar.cam_direct; viewpar.cam_vertical_2 = viewpar.cam_vertical;
                    viewpar.cam_distance_2 = viewpar.cam_distance; viewpar.cam_angle_2 = viewpar.cam_angle;
                    viewpar.cam_pos = viewpar.cam_pos_1; viewpar.cam_direct = viewpar.cam_direct_1; viewpar.cam_vertical = viewpar.cam_vertical_1;
                    viewpar.cam_distance = viewpar.cam_distance_1; viewpar.cam_angle = viewpar.cam_angle_1;
                }
            }
            break;
        case 'E': viewpar.cam_angle += (float)PI * 5.0f / 180.0f; break;
        case 'D': viewpar.cam_angle -= (float)PI * 5.0f / 180.0f; break;
        case 'A':
            viewpar.tracking = 1 - viewpar.tracking;
            if (viewpar.tracking) { viewpar.cam_distance = viewpar.cam_distance_3; viewpar.cam_angle = viewpar.cam_angle_3; }
            else {
                viewpar.cam_distance_3 = viewpar.cam_distance; viewpar.cam_angle_3 = viewpar.cam_angle;
                viewpar.top_view = 0;
                viewpar.cam_pos = viewpar.cam_pos_1; viewpar.cam_direct = viewpar.cam_direct_1; viewpar.cam_vertical = viewpar.cam_vertical_1;
                viewpar.cam_distance = viewpar.cam_distance_1; viewpar.cam_angle = viewpar.cam_angle_1;
            }
            break;
        case 'Z': { viewpar.zoom /= 1.1f; RECT rc; GetClientRect(main_window, &rc); WindowResize(rc.right - rc.left, rc.bottom - rc.top); break; }
        case 'X': { viewpar.zoom *= 1.1f; RECT rc; GetClientRect(main_window, &rc); WindowResize(rc.right - rc.left, rc.bottom - rc.top); break; }
        case VK_ESCAPE: SendMessage(main_window, WM_DESTROY, 0, 0); break;
        }
        break;
    case WM_KEYUP:
        switch (LOWORD(wParam))
        {
        case VK_SHIFT: if_SHIFT_pressed = false; break;
        case VK_SPACE: my_car->breaking_factor = 0.0f; break;
        case VK_UP:    my_car->F = 0.0f; break;
        case VK_DOWN:  my_car->F = 0.0f; break;
        case VK_LEFT:
            if (my_car->if_keep_steer_wheel) my_car->steer_wheel_speed = -0.25f / 8.0f;
            else my_car->steer_wheel_speed = 0;
            my_car->if_keep_steer_wheel = false;
            break;
        case VK_RIGHT:
            if (my_car->if_keep_steer_wheel) my_car->steer_wheel_speed = 0.25f / 8.0f;
            else my_car->steer_wheel_speed = 0;
            my_car->if_keep_steer_wheel = false;
            break;
        }
        break;
    default:
        return DefWindowProc(main_window, message_code, wParam, lParam);
    }
    return 0;
}