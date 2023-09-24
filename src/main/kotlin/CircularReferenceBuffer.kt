import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock

class CircularReferenceBuffer <T>(bufferSize: Int, builder: () -> T) {
	val capacity: Int
	val data: List<T>
	// No atomic integers because of the mutex.
	private val mutex: Mutex
	private var writeHead: Int
	private var readHead: Int

	init {
		this.capacity = bufferSize
		this.data = List(bufferSize) { builder() }
		this.mutex = Mutex()
		this.writeHead = 0
		this.readHead = 0
	}

	/***
	 * Returns a reference to the next element to be read OR null if the buffer is empty.
	 */
	suspend fun readNext(): T? = this.mutex.withLock {
		if(this.readHead != this.writeHead) {
			val readElement = this.data[this.readHead]
			this.readHead = (this.readHead + 1) % this.capacity
			readElement
		} else {
			null
		}
	}

	suspend fun writeNext(): T? = this.mutex.withLock {
		val nextWritePosition = (this.writeHead + 1) % this.capacity
		if(nextWritePosition == this.readHead) {
			null
		} else {
			val writeElement = this.data[nextWritePosition]
			this.writeHead = nextWritePosition
			writeElement
		}
	}
}