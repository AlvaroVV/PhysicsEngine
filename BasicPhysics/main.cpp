/*----------------------------------------------------------------------------------------------------------------------*/
// Fichero main.cpp
/*----------------------------------------------------------------------------------------------------------------------*/

// Includes
#include "DemoApp.h"
#include "windows.h"

/*----------------------------------------------------------------------------------------------------------------------*/

//La declaramos como global para poder hacer referencia con extern.
CDemoApp *g_demoPtr;

/**
*WinMain: Función punto entrada para aplicaciones Windows
*@param:Hinstance -> El sistema operativo lo usa para identificar el ejecutable (EXE) cuando está en memoria.
*@param:PrevInstance -> Ahora no tiene significado, se usaba en los SO de 16 bits, ahora siempre es cero.
*@param:CmdLine-> Contiene los argumentos de los comandos como una cadena de texto Unicode.
*@param:CmdShow -> Un flag para representar cuando la ventana está minimizada, maximizada o mostrada normalmente.
*/
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE hPrevInstance,  LPSTR lpCmdLine, int nCmdShow ) {
	//Estructura de información sobre el mensaje.	
	MSG msg;
	//Windows User Data. Un entero de 64 bits con signo.
	LONGLONG freq, lastTime, currentTime;
	//Recoge la frecuencia de rendimiento.
	QueryPerformanceFrequency( ( LARGE_INTEGER *) &freq );
	float timeScale = 1.0f / freq;
	//Guarda el valor actual del contador de rendimiento.
	QueryPerformanceCounter( ( LARGE_INTEGER * ) &lastTime ); 

	//Inicializamos el puntero a nuestra aplicación
	g_demoPtr = new CDemoApp();

	//Mientras no haya acabado la app, seguimos.
	while( !g_demoPtr->IsFinished() ) {
		//Envía los mensajes entrantes, comprueba la cola de mensajes.
		//Devuelve true si hay un mensaje disponible.
		while( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) ) {
			//Traduce las keys virtuales del mensaje en caracteres, los cuales son envados a la cola de llamadasde mensajes.
			TranslateMessage( &msg );
			//Envía el mensaje a la ventana que tiene el proceso.
			DispatchMessage( &msg );
			//SetCursor(c);
		}
		QueryPerformanceCounter( ( LARGE_INTEGER * ) &currentTime ); 
		//Llamamos al update de la aplicación
		g_demoPtr->Control( ( currentTime - lastTime ) * timeScale );
		lastTime = currentTime;
		//Pintamos nuestra aplicación
		g_demoPtr->Render();
	}
	delete g_demoPtr;
	return static_cast<int>( msg.wParam );
}

/*----------------------------------------------------------------------------------------------------------------------*/

